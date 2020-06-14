#include "RenderAPI.h"
#include "PlatformBase.h"

#if SUPPORT_VULKAN

#include <string.h>
#include <map>
#include <vector>
#include <math.h>

// This plugin does not link to the Vulkan loader, easier to support multiple APIs and systems that don't have Vulkan support
#define VK_NO_PROTOTYPES
#include "Unity/IUnityGraphicsVulkan.h"

#define UNITY_USED_VULKAN_API_FUNCTIONS(apply) \
    apply(vkCreateInstance); \
    apply(vkCmdBeginRenderPass); \
    apply(vkCreateBuffer); \
    apply(vkGetPhysicalDeviceMemoryProperties); \
    apply(vkGetBufferMemoryRequirements); \
    apply(vkMapMemory); \
    apply(vkBindBufferMemory); \
    apply(vkAllocateMemory); \
    apply(vkDestroyBuffer); \
    apply(vkFreeMemory); \
    apply(vkUnmapMemory); \
    apply(vkQueueWaitIdle); \
    apply(vkDeviceWaitIdle); \
    apply(vkCmdCopyBufferToImage); \
    apply(vkFlushMappedMemoryRanges); \
    apply(vkCreatePipelineLayout); \
    apply(vkCreateShaderModule); \
    apply(vkDestroyShaderModule); \
    apply(vkCreateGraphicsPipelines); \
    apply(vkCmdBindPipeline); \
    apply(vkCmdDraw); \
    apply(vkCmdPushConstants); \
    apply(vkCmdBindVertexBuffers); \
    apply(vkDestroyPipeline); \
    apply(vkDestroyPipelineLayout);
    
#define VULKAN_DEFINE_API_FUNCPTR(func) static PFN_##func func
VULKAN_DEFINE_API_FUNCPTR(vkGetInstanceProcAddr);
UNITY_USED_VULKAN_API_FUNCTIONS(VULKAN_DEFINE_API_FUNCPTR);
#undef VULKAN_DEFINE_API_FUNCPTR

static void LoadVulkanAPI(PFN_vkGetInstanceProcAddr getInstanceProcAddr, VkInstance instance)
{
    if (!vkGetInstanceProcAddr && getInstanceProcAddr)
        vkGetInstanceProcAddr = getInstanceProcAddr;

	if (!vkCreateInstance)
		vkCreateInstance = (PFN_vkCreateInstance)vkGetInstanceProcAddr(VK_NULL_HANDLE, "vkCreateInstance");

#define LOAD_VULKAN_FUNC(fn) if (!fn) fn = (PFN_##fn)vkGetInstanceProcAddr(instance, #fn)
    UNITY_USED_VULKAN_API_FUNCTIONS(LOAD_VULKAN_FUNC);
#undef LOAD_VULKAN_FUNC
}

static VKAPI_ATTR void VKAPI_CALL Hook_vkCmdBeginRenderPass(VkCommandBuffer commandBuffer, const VkRenderPassBeginInfo* pRenderPassBegin, VkSubpassContents contents)
{
    // Change this to 'true' to override the clear color with green
	const bool allowOverrideClearColor = false;
    if (pRenderPassBegin->clearValueCount <= 16 && pRenderPassBegin->clearValueCount > 0 && allowOverrideClearColor)
    {
        VkClearValue clearValues[16] = {};
        memcpy(clearValues, pRenderPassBegin->pClearValues, pRenderPassBegin->clearValueCount * sizeof(VkClearValue));

        VkRenderPassBeginInfo patchedBeginInfo = *pRenderPassBegin;
        patchedBeginInfo.pClearValues = clearValues;
        for (unsigned int i = 0; i < pRenderPassBegin->clearValueCount - 1; ++i)
        {
            clearValues[i].color.float32[0] = 0.0;
            clearValues[i].color.float32[1] = 1.0;
            clearValues[i].color.float32[2] = 0.0;
            clearValues[i].color.float32[3] = 1.0;
        }
        vkCmdBeginRenderPass(commandBuffer, &patchedBeginInfo, contents);
    }
    else
    {
        vkCmdBeginRenderPass(commandBuffer, pRenderPassBegin, contents);
    }
}

static VKAPI_ATTR VkResult VKAPI_CALL Hook_vkCreateInstance(const VkInstanceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkInstance* pInstance)
{
    vkCreateInstance = (PFN_vkCreateInstance)vkGetInstanceProcAddr(VK_NULL_HANDLE, "vkCreateInstance");
    VkResult result = vkCreateInstance(pCreateInfo, pAllocator, pInstance);
    if (result == VK_SUCCESS)
        LoadVulkanAPI(vkGetInstanceProcAddr, *pInstance);
 
    return result;
}

static int FindMemoryTypeIndex(VkPhysicalDeviceMemoryProperties const & physicalDeviceMemoryProperties, VkMemoryRequirements const & memoryRequirements, VkMemoryPropertyFlags memoryPropertyFlags)
{
    uint32_t memoryTypeBits = memoryRequirements.memoryTypeBits;

    // Search memtypes to find first index with those properties
    for (uint32_t memoryTypeIndex = 0; memoryTypeIndex < VK_MAX_MEMORY_TYPES; ++memoryTypeIndex)
    {
        if ((memoryTypeBits & 1) == 1)
        {
            // Type is available, does it match user properties?
            if ((physicalDeviceMemoryProperties.memoryTypes[memoryTypeIndex].propertyFlags & memoryPropertyFlags) == memoryPropertyFlags)
                return memoryTypeIndex;
        }
        memoryTypeBits >>= 1;
    }

    return -1;
}

static VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL Hook_vkGetInstanceProcAddr(VkInstance device, const char* funcName)
{
    if (!funcName)
        return NULL;

#define INTERCEPT(fn) if (strcmp(funcName, #fn) == 0) return (PFN_vkVoidFunction)&Hook_##fn
    INTERCEPT(vkCreateInstance);
#undef INTERCEPT

    return NULL;
}

static PFN_vkGetInstanceProcAddr UNITY_INTERFACE_API InterceptVulkanInitialization(PFN_vkGetInstanceProcAddr getInstanceProcAddr, void*)
{
    vkGetInstanceProcAddr = getInstanceProcAddr;
    return Hook_vkGetInstanceProcAddr;
}

extern "C" void RenderAPI_Vulkan_OnPluginLoad(IUnityInterfaces* interfaces)
{
    interfaces->Get<IUnityGraphicsVulkan>()->InterceptInitialization(InterceptVulkanInitialization, NULL);
}

struct VulkanBuffer
{
    VkBuffer buffer;
    VkDeviceMemory deviceMemory;
    void* mapped;
    VkDeviceSize sizeInBytes;
    VkDeviceSize deviceMemorySize;
    VkMemoryPropertyFlags deviceMemoryFlags;
};

class RenderAPI_Vulkan : public RenderAPI
{
public:
    RenderAPI_Vulkan();
    virtual ~RenderAPI_Vulkan() { }

    virtual void ProcessDeviceEvent(UnityGfxDeviceEventType type, IUnityInterfaces* interfaces);
    virtual void* BeginModifyTexture(void* textureHandle, int textureWidth, int textureHeight, int* outRowPitch);
    virtual void EndModifyTexture(void* textureHandle, int textureWidth, int textureHeight, int rowPitch, void* dataPtr);

private:
    typedef std::vector<VulkanBuffer> VulkanBuffers;
    typedef std::map<unsigned long long, VulkanBuffers> DeleteQueue;

private:
    bool CreateVulkanBuffer(size_t bytes, VulkanBuffer* buffer, VkBufferUsageFlags usage);
    void ImmediateDestroyVulkanBuffer(const VulkanBuffer& buffer);
    void SafeDestroy(unsigned long long frameNumber, const VulkanBuffer& buffer);
    void GarbageCollect(bool force = false);

private:
    IUnityGraphicsVulkan* m_UnityVulkan;
    UnityVulkanInstance m_Instance;
    VulkanBuffer m_TextureStagingBuffer;
    VulkanBuffer m_VertexStagingBuffer;
    std::map<unsigned long long, VulkanBuffers> m_DeleteQueue;
    VkPipelineLayout m_TrianglePipelineLayout;
    VkPipeline m_TrianglePipeline;
    VkRenderPass m_TrianglePipelineRenderPass;
};


RenderAPI* CreateRenderAPI_Vulkan()
{
    return new RenderAPI_Vulkan();
}

RenderAPI_Vulkan::RenderAPI_Vulkan()
    : m_UnityVulkan(NULL)
    , m_TextureStagingBuffer()
    , m_VertexStagingBuffer()
    , m_TrianglePipelineLayout(VK_NULL_HANDLE)
    , m_TrianglePipeline(VK_NULL_HANDLE)
    , m_TrianglePipelineRenderPass(VK_NULL_HANDLE)
{
}

void RenderAPI_Vulkan::ProcessDeviceEvent(UnityGfxDeviceEventType type, IUnityInterfaces* interfaces)
{
    switch (type)
    {
    case kUnityGfxDeviceEventInitialize:
        m_UnityVulkan = interfaces->Get<IUnityGraphicsVulkan>();
        m_Instance = m_UnityVulkan->Instance();

        // Make sure Vulkan API functions are loaded
        LoadVulkanAPI(m_Instance.getInstanceProcAddr, m_Instance.instance);

        UnityVulkanPluginEventConfig config_1;
        config_1.graphicsQueueAccess = kUnityVulkanGraphicsQueueAccess_DontCare;
        config_1.renderPassPrecondition = kUnityVulkanRenderPass_EnsureInside;
        config_1.flags = kUnityVulkanEventConfigFlag_EnsurePreviousFrameSubmission | kUnityVulkanEventConfigFlag_ModifiesCommandBuffersState;
        m_UnityVulkan->ConfigureEvent(1, &config_1);

        // alternative way to intercept API
        m_UnityVulkan->InterceptVulkanAPI("vkCmdBeginRenderPass", (PFN_vkVoidFunction)Hook_vkCmdBeginRenderPass);
        break;
    case kUnityGfxDeviceEventShutdown:

        if (m_Instance.device != VK_NULL_HANDLE)
        {
            GarbageCollect(true);
            if (m_TrianglePipeline != VK_NULL_HANDLE)
            {
                vkDestroyPipeline(m_Instance.device, m_TrianglePipeline, NULL);
                m_TrianglePipeline = VK_NULL_HANDLE;
            }
            if (m_TrianglePipelineLayout != VK_NULL_HANDLE)
            {
                vkDestroyPipelineLayout(m_Instance.device, m_TrianglePipelineLayout, NULL);
                m_TrianglePipelineLayout = VK_NULL_HANDLE;
            }
        }

        m_UnityVulkan = NULL;
        m_TrianglePipelineRenderPass = VK_NULL_HANDLE;
        m_Instance = UnityVulkanInstance();

        break;
    }
}


bool RenderAPI_Vulkan::CreateVulkanBuffer(size_t sizeInBytes, VulkanBuffer* buffer, VkBufferUsageFlags usage)
{
    if (sizeInBytes == 0)
        return false;

    VkBufferCreateInfo bufferCreateInfo;
    bufferCreateInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferCreateInfo.pNext = NULL;
    bufferCreateInfo.pQueueFamilyIndices = &m_Instance.queueFamilyIndex;
    bufferCreateInfo.queueFamilyIndexCount = 1;
    bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    bufferCreateInfo.usage = usage;
    bufferCreateInfo.flags = 0;
    bufferCreateInfo.size = sizeInBytes;

    *buffer = VulkanBuffer();

    if (vkCreateBuffer(m_Instance.device, &bufferCreateInfo, NULL, &buffer->buffer) != VK_SUCCESS)
        return false;

    VkPhysicalDeviceMemoryProperties physicalDeviceProperties;
    vkGetPhysicalDeviceMemoryProperties(m_Instance.physicalDevice, &physicalDeviceProperties);

    VkMemoryRequirements memoryRequirements;
    vkGetBufferMemoryRequirements(m_Instance.device, buffer->buffer, &memoryRequirements);

    const int memoryTypeIndex = FindMemoryTypeIndex(physicalDeviceProperties, memoryRequirements, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    if (memoryTypeIndex < 0)
    {
        ImmediateDestroyVulkanBuffer(*buffer);
        return false;
    }

    VkMemoryAllocateInfo memoryAllocateInfo;
    memoryAllocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    memoryAllocateInfo.pNext = NULL;
    memoryAllocateInfo.memoryTypeIndex = memoryTypeIndex;
    memoryAllocateInfo.allocationSize = memoryRequirements.size;

    if (vkAllocateMemory(m_Instance.device, &memoryAllocateInfo, NULL, &buffer->deviceMemory) != VK_SUCCESS)
    {
        ImmediateDestroyVulkanBuffer(*buffer);
        return false;
    }

    if (vkMapMemory(m_Instance.device, buffer->deviceMemory, 0, VK_WHOLE_SIZE, 0, &buffer->mapped) != VK_SUCCESS)
    {
        ImmediateDestroyVulkanBuffer(*buffer);
        return false;
    }

    if (vkBindBufferMemory(m_Instance.device, buffer->buffer, buffer->deviceMemory, 0) != VK_SUCCESS)
    {
        ImmediateDestroyVulkanBuffer(*buffer);
        return false;
    }

    buffer->sizeInBytes = sizeInBytes;
    buffer->deviceMemoryFlags = physicalDeviceProperties.memoryTypes[memoryTypeIndex].propertyFlags;
    buffer->deviceMemorySize = memoryAllocateInfo.allocationSize;

    return true;
}

void RenderAPI_Vulkan::ImmediateDestroyVulkanBuffer(const VulkanBuffer& buffer)
{
    if (buffer.buffer != VK_NULL_HANDLE)
        vkDestroyBuffer(m_Instance.device, buffer.buffer, NULL);

    if (buffer.mapped && buffer.deviceMemory != VK_NULL_HANDLE)
        vkUnmapMemory(m_Instance.device, buffer.deviceMemory);

    if (buffer.deviceMemory != VK_NULL_HANDLE)
        vkFreeMemory(m_Instance.device, buffer.deviceMemory, NULL);
}


void RenderAPI_Vulkan::SafeDestroy(unsigned long long frameNumber, const VulkanBuffer& buffer)
{
    m_DeleteQueue[frameNumber].push_back(buffer);
}

void RenderAPI_Vulkan::GarbageCollect(bool force /*= false*/)
{
    UnityVulkanRecordingState recordingState;
    if (force)
        recordingState.safeFrameNumber = ~0ull;
    else
        if (!m_UnityVulkan->CommandRecordingState(&recordingState, kUnityVulkanGraphicsQueueAccess_DontCare))
            return;

    DeleteQueue::iterator it = m_DeleteQueue.begin();
    while (it != m_DeleteQueue.end())
    {
        if (it->first <= recordingState.safeFrameNumber)
        {
            for (size_t i = 0; i < it->second.size(); ++i)
                ImmediateDestroyVulkanBuffer(it->second[i]);
            m_DeleteQueue.erase(it++);
        }
        else
            ++it;
    }
}

void RenderAPI_Vulkan::DrawSimpleTriangles(const float worldMatrix[16], int triangleCount, const void* verticesFloat3Byte4)
{
     // not needed, we already configured the event to be inside a render pass
     //   m_UnityVulkan->EnsureInsideRenderPass();

    UnityVulkanRecordingState recordingState;
    if (!m_UnityVulkan->CommandRecordingState(&recordingState, kUnityVulkanGraphicsQueueAccess_DontCare))
        return;

    // Unity does not destroy render passes, so this is safe regarding ABA-problem
    if (recordingState.renderPass != m_TrianglePipelineRenderPass)
    {
        if (m_TrianglePipelineLayout == VK_NULL_HANDLE)
            m_TrianglePipelineLayout = CreateTrianglePipelineLayout(m_Instance.device);

        m_TrianglePipeline = CreateTrianglePipeline(m_Instance.device, m_TrianglePipelineLayout, recordingState.renderPass, VK_NULL_HANDLE);
		m_TrianglePipelineRenderPass = recordingState.renderPass;
    }

    if (m_TrianglePipeline != VK_NULL_HANDLE && m_TrianglePipelineLayout != VK_NULL_HANDLE)
    {
        VulkanBuffer buffer;
        if (!CreateVulkanBuffer(16 * 3 * triangleCount, &buffer, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT))
            return;

        memcpy(buffer.mapped, verticesFloat3Byte4, static_cast<size_t>(buffer.sizeInBytes));
        if (!(buffer.deviceMemoryFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT))
        {
            VkMappedMemoryRange range;
            range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
            range.pNext = NULL;
            range.memory = buffer.deviceMemory;
            range.offset = 0;
            range.size = buffer.deviceMemorySize;
            vkFlushMappedMemoryRanges(m_Instance.device, 1, &range);
        }

        const VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(recordingState.commandBuffer, 0, 1, &buffer.buffer, &offset);
        vkCmdPushConstants(recordingState.commandBuffer, m_TrianglePipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, 64, (const void*)worldMatrix);
        vkCmdBindPipeline(recordingState.commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_TrianglePipeline);
        vkCmdDraw(recordingState.commandBuffer, triangleCount * 3, 1, 0, 0);

        SafeDestroy(recordingState.currentFrameNumber, buffer);
    }

    GarbageCollect();
}

void* RenderAPI_Vulkan::BeginModifyTexture(void* textureHandle, int textureWidth, int textureHeight, int* outRowPitch)
{
    *outRowPitch = textureWidth * 4;
    const size_t stagingBufferSizeRequirements = *outRowPitch * textureHeight;

    UnityVulkanRecordingState recordingState;
    if (!m_UnityVulkan->CommandRecordingState(&recordingState, kUnityVulkanGraphicsQueueAccess_DontCare))
        return NULL;

    SafeDestroy(recordingState.currentFrameNumber, m_TextureStagingBuffer);
    m_TextureStagingBuffer = VulkanBuffer();
    if (!CreateVulkanBuffer(stagingBufferSizeRequirements, &m_TextureStagingBuffer, VK_BUFFER_USAGE_TRANSFER_SRC_BIT))
        return NULL;

    return m_TextureStagingBuffer.mapped;
}

void RenderAPI_Vulkan::EndModifyTexture(void* textureHandle, int textureWidth, int textureHeight, int rowPitch, void* dataPtr)
{
    // cannot do resource uploads inside renderpass
    m_UnityVulkan->EnsureOutsideRenderPass();

    UnityVulkanImage image;
    if (!m_UnityVulkan->AccessTexture(textureHandle, UnityVulkanWholeImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT, kUnityVulkanResourceAccess_PipelineBarrier, &image))
        return;

    UnityVulkanRecordingState recordingState;
    if (!m_UnityVulkan->CommandRecordingState(&recordingState, kUnityVulkanGraphicsQueueAccess_DontCare))
        return;

    VkBufferImageCopy region;
    region.bufferImageHeight = 0;
    region.bufferRowLength = 0;
    region.bufferOffset = 0;
    region.imageOffset.x = 0;
    region.imageOffset.y = 0;
    region.imageOffset.z = 0;
    region.imageExtent.width = textureWidth;
    region.imageExtent.height = textureHeight;
    region.imageExtent.depth = 1;
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.baseArrayLayer = 0;
    region.imageSubresource.layerCount = 1;
    region.imageSubresource.mipLevel = 0;
    vkCmdCopyBufferToImage(recordingState.commandBuffer, m_TextureStagingBuffer.buffer, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
}


#endif // #if SUPPORT_VULKAN
