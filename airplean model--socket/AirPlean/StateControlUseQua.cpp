//=============================================================================
// Desc: 文件网格模型的使用
//=============================================================================
#include <d3dx9.h>
//#include <afxsock.h>
//#include <afxwin.h>


//-----------------------------------------------------------------------------
// Desc: 全局变量
//-----------------------------------------------------------------------------
LPDIRECT3D9             g_pD3D           = NULL;  //Direct3D对象
LPDIRECT3DDEVICE9       g_pd3dDevice     = NULL;  //Direct3D设备对象

LPD3DXMESH              g_pMesh          = NULL;  //网格模型对象
D3DMATERIAL9*           g_pMeshMaterials = NULL;  //网格模型材质
LPDIRECT3DTEXTURE9*     g_pMeshTextures  = NULL;  //网格模型纹理
DWORD                   g_dwNumMaterials = 0L;    //网格模型材质数量
D3DXVECTOR3             g_vCenter;                //网格模型中心点 

BYTE  m_bKey[256];			// keyboard state buffer

D3DXMATRIX              g_matWorld;

SOCKET ListenSocket;//套接字
HANDLE  hThread; //线程的句柄，线程用于接收串口消息

float q0=0, q1=0, q2=0, q3=0;
float q0q0=0, q1q1=0, q2q2=0, q3q3=0;
D3DXQUATERNION qR;//四元数

float fRoll_pre = 0.0f, fPitch_pre = 0.0f, fYaw_pre = 0.0f;//全局欧拉角
float fRoll_temp = 0.0f, fPitch_temp = 0.0f, fYaw_temp = 0.0f;//本次计算得到的欧拉角
float fRoll_loop = 0.0f, fPitch_loop = 0.0f, fYaw_loop = 0.0f;//这一秒钟变化的欧拉角，是这一次和上一次的差值

DWORD pre_frame_time = 0;
DWORD pre_second_time = 0;

VOID Render();

//-----------------------------------------------------------------------------
// Desc: 设置世界矩阵
//-----------------------------------------------------------------------------
VOID SetWorldMatrix()
{
	static long curTime=0;
	static float elapsetime=0;
	elapsetime = (timeGetTime()-curTime)/1000.0f;  //两次调用次函数的时间间隔，以s为单位
	curTime = timeGetTime();

	//创建并设置世界矩阵  
	float fRoll = 0.0f, fPitch = 0.0f, fYaw = 0.0f;//本次和上次的欧拉角差值
	/*
	if (BUF[0] - 48) fRoll  -= 3*elapsetime;
    if (BUF[1] - 48) fRoll  += 3*elapsetime;
	if (BUF[2] - 48) fPitch -= 3*elapsetime;
    if (BUF[3] - 48) fPitch += 3*elapsetime;
	if (BUF[4] - 48) fYaw   -= 3*elapsetime;
    if (BUF[5] - 48) fYaw   += 3*elapsetime;
	*/

	/*
	if (m_bKey['D']) fRoll  -= 3*elapsetime;
    if (m_bKey['A']) fRoll  += 3*elapsetime;
	if (m_bKey['S']) fPitch -= 3*elapsetime;
    if (m_bKey['W']) fPitch += 3*elapsetime;
	if (m_bKey['Q']) fYaw   -= 3*elapsetime;
    if (m_bKey['E']) fYaw   += 3*elapsetime;
	*/

	//fYaw   = (fYaw_temp   - fYaw_pre)   / 33;  //偏航
	//fPitch = (fPitch_temp - fPitch_pre) / 33;  //俯仰
	fRoll  = (-fRoll_temp  - fPitch_pre) / 50;// 横滚
	if (fRoll < 0.01 && fRoll > -0.01) 
	{
		fRoll = 0;
	}
	fPitch = (-fPitch_temp - fPitch_pre) / 1;
	if (fPitch < 0.01 && fPitch > -0.01)
	{
		fPitch = 0;
	}

	fYaw_pre   = -fYaw_temp;
	fPitch_pre = -fPitch_temp;
	fRoll_pre  = fRoll_temp;
	
	//更新网格模型姿态
	D3DXMATRIX matRot;//矩阵
	D3DXQuaternionRotationYawPitchRoll (&qR, fYaw, fPitch, fRoll);	//欧拉角转换为四元数
	
	//qR.w = 0;
	//qR.x = 0;
	//qR.y = 0.1;
	//qR.z = 0;
	D3DXMatrixRotationQuaternion (&matRot, &qR);  //四元数转换为矩阵
	D3DXMatrixMultiply (&g_matWorld, &matRot, &g_matWorld);  //两个四元矩阵相乘


	/* 第一行是左右， 第二行是上下，第三行是前后 */
	/*
	//获取网格模型前向量
	static D3DXVECTOR3 vLook;
	vLook.x = g_matWorld._31;
	vLook.y = g_matWorld._32;
	vLook.z = g_matWorld._33;
	*/

	
	static D3DXVECTOR3 vLook;//左右
	vLook.x = g_matWorld._11;
	vLook.y = g_matWorld._12;
	vLook.z = g_matWorld._13;

	static D3DXVECTOR3 vLook1;//上下
	vLook1.x = g_matWorld._21;
	vLook1.y = g_matWorld._22;
	vLook1.z = g_matWorld._23;

	static D3DXVECTOR3 vLook2;//前后
	vLook2.x = g_matWorld._31;
	vLook2.y = g_matWorld._32;
	vLook2.z = g_matWorld._33;


	//向左移动
    if (m_bKey['G'])
	{
		g_matWorld._41 += 10*elapsetime * vLook.x;
		g_matWorld._42 += 10*elapsetime * vLook.y;
		g_matWorld._43 += 10*elapsetime * vLook.z;
	}

	//向右移动
    if (m_bKey['B']) 
	{
		g_matWorld._41 -= 10*elapsetime * vLook.x;
		g_matWorld._42 -= 10*elapsetime * vLook.y;
		g_matWorld._43 -= 10*elapsetime * vLook.z;
	}
	//向上移动
    if (m_bKey['F'])
	{
		g_matWorld._41 += 10*elapsetime * vLook1.x;
		g_matWorld._42 += 10*elapsetime * vLook1.y;
		g_matWorld._43 += 10*elapsetime * vLook1.z;
	}

	//向下移动
    if (m_bKey['V']) 
	{
		g_matWorld._41 -= 10*elapsetime * vLook1.x;
		g_matWorld._42 -= 10*elapsetime * vLook1.y;
		g_matWorld._43 -= 10*elapsetime * vLook1.z;
	}

	//向前移动
    if (m_bKey['H'])
	{
		g_matWorld._41 += 10*elapsetime * vLook2.x;
		g_matWorld._42 += 10*elapsetime * vLook2.y;
		g_matWorld._43 += 10*elapsetime * vLook2.z;
	}

	//向后移动
    if (m_bKey['N']) 
	{
		g_matWorld._41 -= 10*elapsetime * vLook2.x;
		g_matWorld._42 -= 10*elapsetime * vLook2.y;
		g_matWorld._43 -= 10*elapsetime * vLook2.z;
	}
	

    g_pd3dDevice->SetTransform( D3DTS_WORLD, &g_matWorld );
}


//-----------------------------------------------------------------------------
// Desc: 设置观察矩阵和投影矩阵
//-----------------------------------------------------------------------------
VOID SetViewAndProjMatrix()
{
    //创建并设置观察矩阵
    D3DXVECTOR3 vEyePt( 0.0f, 0.0f,-20.0f );
    D3DXVECTOR3 vLookatPt( 0.0f, 0.0f, 0.0f );
    D3DXVECTOR3 vUpVec( 0.0f, 1.0f, 0.0f );
    D3DXMATRIXA16 matView;
    D3DXMatrixLookAtLH( &matView, &vEyePt, &vLookatPt, &vUpVec );
    g_pd3dDevice->SetTransform( D3DTS_VIEW, &matView );

    //创建并设置投影矩阵
    D3DXMATRIXA16 matProj;
    D3DXMatrixPerspectiveFovLH( &matProj, D3DX_PI/4, 1.0f, 1.0f, 500.0f );
    g_pd3dDevice->SetTransform( D3DTS_PROJECTION, &matProj );
}

//-----------------------------------------------------------------------------
// Desc: 初始化网络套接字
//-----------------------------------------------------------------------------
bool Initsocket()
{
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;
	struct sockaddr_in remoteaddr;

	//在使用之前，必须制定一个winsock版本，使用之后释放
	wVersionRequested = MAKEWORD(2, 2);
	err = WSAStartup(wVersionRequested, &wsaData);

	//创建一个TCP套接字
	ListenSocket=socket(PF_INET, SOCK_STREAM, 0);

	//建立本地套接字与服务器的连接
	remoteaddr.sin_family = AF_INET;
	remoteaddr.sin_port = htons(8080);
	remoteaddr.sin_addr.s_addr = inet_addr("192.168.11.254");
	connect(ListenSocket, (SOCKADDR *)&remoteaddr, sizeof(remoteaddr));

	return TRUE;
}

//-----------------------------------------------------------------------------
// Desc: 初始化Direct3D
//-----------------------------------------------------------------------------
HRESULT InitD3D( HWND hWnd )
{
	//创建Direct3D对象, 该对象用于创建Direct3D设备对象
    if( NULL == ( g_pD3D = Direct3DCreate9( D3D_SDK_VERSION ) ) )
        return E_FAIL;

	//设置D3DPRESENT_PARAMETERS结构, 准备创建Direct3D设备对象
    D3DPRESENT_PARAMETERS d3dpp; 
    ZeroMemory( &d3dpp, sizeof(d3dpp) );
    d3dpp.Windowed = TRUE;
    d3dpp.SwapEffect = D3DSWAPEFFECT_DISCARD;
    d3dpp.BackBufferFormat = D3DFMT_UNKNOWN;
    d3dpp.EnableAutoDepthStencil = TRUE;
    d3dpp.AutoDepthStencilFormat = D3DFMT_D16;

    //创建Direct3D设备对象
    if( FAILED( g_pD3D->CreateDevice( D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd,
                                      D3DCREATE_SOFTWARE_VERTEXPROCESSING,
                                      &d3dpp, &g_pd3dDevice ) ) )
    {
        return E_FAIL;
    }

	//设置纹理过滤状态
	g_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG1, D3DTA_TEXTURE );
    g_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP,   D3DTOP_SELECTARG1 );
    g_pd3dDevice->SetSamplerState( 0, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
    g_pd3dDevice->SetSamplerState( 0, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );

	//设置观察矩阵和投影矩阵
	SetViewAndProjMatrix();

	ZeroMemory( m_bKey, 256 );
	D3DXMatrixIdentity(&g_matWorld);
	
    return S_OK;
}


//-----------------------------------------------------------------------------
// Desc: 从绝对路径中提取纹理文件名
//-----------------------------------------------------------------------------
void RemovePathFromFileName(LPSTR fullPath, LPWSTR fileName)
{
	//先将fullPath的类型变换为LPWSTR
	WCHAR wszBuf[MAX_PATH];
	MultiByteToWideChar( CP_ACP, 0, fullPath, -1, wszBuf, MAX_PATH );
	wszBuf[MAX_PATH-1] = L'\0';

	WCHAR* wszFullPath = wszBuf;

	//从绝对路径中提取文件名
	LPWSTR pch=wcsrchr(wszFullPath,'\\');
	if (pch)
		lstrcpy(fileName, ++pch);
	else
		lstrcpy(fileName, wszFullPath);
}



//-----------------------------------------------------------------------------
// Desc: 创建场景图形
//-----------------------------------------------------------------------------
HRESULT InitGeometry()
{
    LPD3DXBUFFER pD3DXMtrlBuffer;  //存储网格模型材质的缓冲区对象

    //从磁盘文件加载网格模型
    if( FAILED( D3DXLoadMeshFromX( L"airplane.x", D3DXMESH_MANAGED, 
                                   g_pd3dDevice, NULL, 
                                   &pD3DXMtrlBuffer, NULL, &g_dwNumMaterials, 
                                   &g_pMesh ) ) )
    {
        MessageBox(NULL, L"Could not find airplane.x", L"Mesh", MB_OK);
        return E_FAIL;
    }

    //从网格模型中提取材质属性和纹理文件名 
    D3DXMATERIAL* d3dxMaterials = (D3DXMATERIAL*)pD3DXMtrlBuffer->GetBufferPointer();
    g_pMeshMaterials = new D3DMATERIAL9[g_dwNumMaterials];

    if( g_pMeshMaterials == NULL )
        return E_OUTOFMEMORY;

    g_pMeshTextures  = new LPDIRECT3DTEXTURE9[g_dwNumMaterials];
    if( g_pMeshTextures == NULL )
        return E_OUTOFMEMORY;

	//逐块提取网格模型材质属性和纹理文件名
    for( DWORD i=0; i<g_dwNumMaterials; i++ )
    {
        //材料属性
        g_pMeshMaterials[i] = d3dxMaterials[i].MatD3D;
		//设置模型材料的环境光反射系数, 因为模型材料本身没有设置环境光反射系数
        g_pMeshMaterials[i].Ambient = g_pMeshMaterials[i].Diffuse;

        g_pMeshTextures[i] = NULL;
        if( d3dxMaterials[i].pTextureFilename != NULL && 
            strlen(d3dxMaterials[i].pTextureFilename) > 0 )
        {
			//获取纹理文件名
			WCHAR filename[256];
			RemovePathFromFileName(d3dxMaterials[i].pTextureFilename, filename);

            //创建纹理
            if( FAILED( D3DXCreateTextureFromFile( g_pd3dDevice, filename, 
                                                  &g_pMeshTextures[i] ) ) )
            {
                MessageBox(NULL, L"Could not find texture file", L"Mesh", MB_OK);
            }
        }
    }

	//释放在加载模型文件时创建的保存模型材质和纹理数据的缓冲区对象
    pD3DXMtrlBuffer->Release();

	//计算网格模型边界球中心
	LPDIRECT3DVERTEXBUFFER9 pVB; //网格对象顶点缓冲区接口
    if( SUCCEEDED( g_pMesh->GetVertexBuffer( &pVB ) ) )
    {
        struct VERTEX { FLOAT x,y,z,tu,tv; };
        VERTEX*      pVertices; //顶点缓冲区指针
		float        fRadius;

        pVB->Lock( 0, 0, (void**)&pVertices, 0 ); 

		//计算网格对象的边界球中心和半径
		D3DXComputeBoundingSphere( (D3DXVECTOR3*)pVertices, g_pMesh->GetNumVertices(),
                                    D3DXGetFVFVertexSize(g_pMesh->GetFVF()),
                                    &g_vCenter, &fRadius );
		pVB->Unlock();
        pVB->Release();
	}

    return S_OK;
}


//-----------------------------------------------------------------------------
// Desc: 释放创建的对象
//-----------------------------------------------------------------------------
VOID Cleanup()
{
	//释放网格模型材质
    if( g_pMeshMaterials != NULL ) 
        delete[] g_pMeshMaterials;

	//释放网格模型纹理
    if( g_pMeshTextures )
    {
        for( DWORD i = 0; i < g_dwNumMaterials; i++ )
        {
            if( g_pMeshTextures[i] )
                g_pMeshTextures[i]->Release();
        }
        delete[] g_pMeshTextures;
    }

	//释放网格模型对象
    if( g_pMesh != NULL )
        g_pMesh->Release();
    
	//释放Direct3D设备对象
    if( g_pd3dDevice != NULL )
        g_pd3dDevice->Release();

	//释放Direct3D对象
    if( g_pD3D != NULL )
        g_pD3D->Release();

	//程序退出时释放套接字和线程
	(void)closesocket(ListenSocket);
	WSACleanup();
	CloseHandle(hThread);
}


//-----------------------------------------------------------------------------
// Desc: 渲染场景
//-----------------------------------------------------------------------------
VOID Render()
{
    // 清除缓冲区
    g_pd3dDevice->Clear( 0, NULL, D3DCLEAR_TARGET|D3DCLEAR_ZBUFFER, 
                         D3DCOLOR_XRGB(0,0,255), 1.0f, 0 );
    
    //开始渲染场景
    if( SUCCEEDED( g_pd3dDevice->BeginScene() ) )
    {
        SetWorldMatrix();  //设置世界矩阵

        //逐块渲染网格模型
        for( DWORD i=0; i<g_dwNumMaterials; i++ )
        {
            //设置材料和纹理
            g_pd3dDevice->SetMaterial( &g_pMeshMaterials[i] );
            g_pd3dDevice->SetTexture( 0, g_pMeshTextures[i] );

			//渲染模型
            g_pMesh->DrawSubset( i );
        }

        //场景渲染结束
        g_pd3dDevice->EndScene();
    }

    //在屏幕上显示场景
    g_pd3dDevice->Present( NULL, NULL, NULL, NULL );
}


//-----------------------------------------------------------------------------
// Desc: 窗口过程, 处理消息
//-----------------------------------------------------------------------------
LRESULT WINAPI MsgProc( HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam )
{
    switch( msg )
    {
	case WM_DESTROY:
		Cleanup();
		PostQuitMessage( 0 );
		return 0;

	case WM_KEYDOWN:
		m_bKey[wParam] = 1;
		return 0;

	case WM_KEYUP:
		m_bKey[wParam] = 0;
		return 0;
    }

    return DefWindowProc( hWnd, msg, wParam, lParam );
}

//-----------------------------------------------------------------------------
// Desc: 读串口数据
//-----------------------------------------------------------------------------
DWORD WINAPI thread_read_socket(LPVOID lpParam) 
//int thread_read_uart()
{
	BYTE buffer[32] = {0};
	BYTE i = 0;
	BOOL bResult = TRUE;
	DWORD BytesRead = 0;
	BYTE BytesConut = 0;
	BYTE *uchar = 0;

	while(1)
	{
		if(recv(ListenSocket, (char *)buffer, 32, 0))
		{
			//qR.y = 0.1;
		}
		//bResult = ReadFile(hCom, buffer, 30, &BytesRead, NULL);
						
		for (i = 0; i< 30; i++)
		{
			if (0xAA == buffer[i] && i < 16)  //检查帧头和帧尾
			{
				if (0x55 == buffer[i+1] && 0x0C == buffer[i+14])
				{
					//qR.y = 0.1;
					uchar = (BYTE *)&fYaw_temp;
					*(uchar + 3) = buffer[i+2];
					*(uchar + 2) = buffer[i+3];
					*(uchar + 1) = buffer[i+4];
					*uchar       = buffer[i+5];

					uchar = (BYTE *)&fPitch_temp;
					*(uchar + 3) = buffer[i+6];
					*(uchar + 2) = buffer[i+7];
					*(uchar + 1) = buffer[i+8];
					*uchar       = buffer[i+9];

					uchar = (BYTE *)&fRoll_temp;
					*(uchar + 3) = buffer[i+10];
					*(uchar + 2) = buffer[i+11];
					*(uchar + 1) = buffer[i+12];
					*uchar       = buffer[i+13];

					//fYaw_temp   = (float)(buffer[i+2]<<24 | buffer[i+3]<<16 | buffer[i+4]<<8 | buffer[i+5]);     //偏航角 全部为弧度，需要的也是弧度
					//fPitch_temp = (float)(buffer[i+6]<<24 | buffer[i+7]<<16 | buffer[i+8]<<8 | buffer[i+9]);     //俯仰角
					//fRoll_temp  = (float)(buffer[i+10]<<24 | buffer[i+11]<<16 | buffer[i+12]<<8 | buffer[i+13]); //滚动角

					break;
					
				}
			}
		}
		Sleep(15); //改为30ms钟读一次试试		
	}

	return 0;
}

//-----------------------------------------------------------------------------
// Desc: 入口函数
//-----------------------------------------------------------------------------
INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR, INT )
{
    //注册窗口类
    WNDCLASSEX wc = { sizeof(WNDCLASSEX), CS_CLASSDC, MsgProc, 0L, 0L, 
                      GetModuleHandle(NULL), NULL, NULL, NULL, NULL,
                      L"ClassName", NULL };
    RegisterClassEx( &wc );

    //创建窗口
    HWND hWnd = CreateWindow( L"ClassName", L"通过四元数实现模型空间定位", 
                              WS_OVERLAPPEDWINDOW, 200, 100, 500, 500,
                              GetDesktopWindow(), NULL, wc.hInstance, NULL );

	//初始化网络套接字
	Initsocket();

    //创建一个线程，用于读取网口数据；
	DWORD dwThreadParam, dwThreadID;
	hThread=CreateThread(NULL, 0, thread_read_socket, &dwThreadParam, 0, &dwThreadID);

	//初始化Direct3D
    if( SUCCEEDED( InitD3D( hWnd ) ) )
    { 
		//创建场景图形
        if( SUCCEEDED( InitGeometry() ) )
        {
            //显示窗口
            ShowWindow( hWnd, SW_SHOWDEFAULT );
            UpdateWindow( hWnd );

            //进入消息循环
            MSG msg; 
            ZeroMemory( &msg, sizeof(msg) );
            while( msg.message!=WM_QUIT )
            {
                if( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE ) )//在没有消息的时候，不会忙等，而是返回0值
                {
                    TranslateMessage( &msg );
                    DispatchMessage( &msg );
                }
                else
				{
					if ((timeGetTime() - pre_frame_time) > 25)//设置每秒40帧的速率渲染
					{
						Render();  //渲染场景
						pre_frame_time = timeGetTime() - 25;
					}

				}
            }
        }
    }

    UnregisterClass( L"ClassName", wc.hInstance );

	return 0;
}
