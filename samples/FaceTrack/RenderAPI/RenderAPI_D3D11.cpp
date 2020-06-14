#include "RenderAPI.h"
#include "PlatformBase.h"

// Direct3D 11 implementation of RenderAPI.

#if SUPPORT_D3D11

#include <assert.h>
#include <d3d11.h>
#include "Unity/IUnityGraphicsD3D11.h"


class RenderAPI_D3D11 : public RenderAPI
{
public:
	RenderAPI_D3D11();
	virtual ~RenderAPI_D3D11() { }

	virtual void ProcessDeviceEvent(UnityGfxDeviceEventType type, IUnityInterfaces* interfaces);

	virtual void* BeginModifyTexture(void* textureHandle, int textureWidth, int textureHeight, int* outRowPitch);
	virtual void EndModifyTexture(void* textureHandle, int textureWidth, int textureHeight, int rowPitch, void* dataPtr);

private:
	void CreateResources();
	void ReleaseResources();

private:
	ID3D11Device* m_Device;
};


RenderAPI* CreateRenderAPI_D3D11()
{
	return new RenderAPI_D3D11();
}




RenderAPI_D3D11::RenderAPI_D3D11()
	: m_Device(NULL)
{
}


void RenderAPI_D3D11::ProcessDeviceEvent(UnityGfxDeviceEventType type, IUnityInterfaces* interfaces)
{
	switch (type)
	{
	case kUnityGfxDeviceEventInitialize:
	{
		IUnityGraphicsD3D11* d3d = interfaces->Get<IUnityGraphicsD3D11>();
		m_Device = d3d->GetDevice();
		CreateResources();
		break;
	}
	case kUnityGfxDeviceEventShutdown:
		ReleaseResources();
		break;
	}
}


void RenderAPI_D3D11::CreateResources()
{
}


void RenderAPI_D3D11::ReleaseResources()
{
}



void* RenderAPI_D3D11::BeginModifyTexture(void* textureHandle, int textureWidth, int textureHeight, int* outRowPitch)
{

	ID3D11Texture2D* d3dtex = (ID3D11Texture2D*)textureHandle;
	assert(d3dtex);

	ID3D11DeviceContext* ctx = NULL;
	m_Device->GetImmediateContext(&ctx);


	ctx->Release();
	
	//ID3D11Texture2D* d3dtex = (ID3D11Texture2D*)textureHandle;
	//D3D11_TEXTURE2D_DESC desc;
	//d3dtex->GetDesc(&desc);

	/*
	ID3D11Texture2D* pNewTexture = NULL;
	D3D11_TEXTURE2D_DESC description;
	d3dtex->GetDesc(&description);

	description.BindFlags = 0;

	description.CPUAccessFlags = D3D11_CPU_ACCESS_READ | D3D11_CPU_ACCESS_WRITE;
	description.Usage = D3D11_USAGE_STAGING;
	HRESULT hr = m_Device->CreateTexture2D(&description, NULL, &pNewTexture);

	ID3D11DeviceContext* ctx = NULL;
	m_Device->GetImmediateContext(&ctx);

	ctx->CopyResource(pNewTexture, d3dtex);

	D3D11_MAPPED_SUBRESOURCE resource;
	UINT subresource = D3D11CalcSubresource(0, 0, 0);
	ctx->Map(pNewTexture, subresource, D3D11_MAP_READ_WRITE, 0, &resource);


	// COPY from texture to bitmap buffer
	uint8_t* sptr = reinterpret_cast<uint8_t*>(resource.pData);
	uint8_t* dptr = new uint8_t[desc.Width * desc.Height * 4];

	for (size_t h = 0; h < desc.Height; ++h)
	{
		size_t msize = std::min<size_t>(desc.Width * 4, resource.RowPitch);
		memcpy_s(dptr, desc.Width * 4, sptr, msize);
		sptr += resource.RowPitch;
		dptr += desc.Width * 4;
	}

	dptr -= desc.Width * desc.Height * 4;

	// SWAP BGR to RGB bitmap
	uint32_t* dPtr = reinterpret_cast<uint32_t*>(dptr);
	for (size_t count = 0; count < desc.Width * desc.Height * 4; count += 4)
	{
		uint32_t t = *(dPtr);

		uint32_t t1 = (t & 0x00ff0000) >> 16;
		uint32_t t2 = (t & 0x000000ff) << 16;
		uint32_t t3 = (t & 0x0000ff00);
		uint32_t ta = (t & 0xFF000000);

		*(dPtr++) = t1 | t2 | t3 | ta;
	}
	
	*/

	const int rowPitch = textureWidth * 4;
	// Just allocate a system memory buffer here for simplicity
	unsigned char* data = new unsigned char[rowPitch * textureHeight];
	*outRowPitch = rowPitch;
	return data;
}


void RenderAPI_D3D11::EndModifyTexture(void* textureHandle, int textureWidth, int textureHeight, int rowPitch, void* dataPtr)
{
	ID3D11Texture2D* d3dtex = (ID3D11Texture2D*)textureHandle;
	assert(d3dtex);

	ID3D11DeviceContext* ctx = NULL;
	m_Device->GetImmediateContext(&ctx);
	// Update texture data, and free the memory buffer
	ctx->UpdateSubresource(d3dtex, 0, NULL, dataPtr, rowPitch, 0);
	delete[] (unsigned char*)dataPtr;
	ctx->Release();
}



#endif // #if SUPPORT_D3D11
