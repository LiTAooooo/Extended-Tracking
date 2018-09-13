// ============================================================================
//	Includes
// ============================================================================


//====ORB-SLAM2 includes=====

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

// ============================



#ifdef _WIN32
#  include <windows.h>
#endif
#include <stdio.h>
#ifdef _WIN32
#  define snprintf _snprintf
#endif
#include <string.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif


#include <AR/ar.h>
#include <AR/arMulti.h>
#include <AR/video.h>
#include <AR/gsub_lite.h>
#include <AR/arFilterTransMat.h>
#include <AR2/tracking.h>

#include "ARMarkerNFT.h"
#include "trackingSub.h"
#include <KPM/kpm.h>



// ============================================================================
//	Constants
// ============================================================================

#define PAGES_MAX               10       // Maximum number of pages expected. You can change this down (to save memory) or up (to accomodate more pages.)

#define VIEW_SCALEFACTOR		1.0		// Units received from ARToolKit tracking will be multiplied by this factor before being used in OpenGL drawing.
#define VIEW_DISTANCE_MIN		10.0		// Objects closer to the camera than this will not be displayed. OpenGL units.
#define VIEW_DISTANCE_MAX		10000.0		// Objects further away from the camera than this will not be displayed. OpenGL units.

// ============================================================================
//	Global variables
// ============================================================================


// ===============ORB-SLAM2=================

    ORB_SLAM2::System *ptr_SLAM;
    ORB_SLAM2::System::my_eTrackingState *ptr_State;	//用于标记SLAM系统的状态
    
// =========================================


// Preferences.
static int prefWindowed = TRUE;
static int prefWidth = 640;				// Fullscreen mode width.
static int prefHeight = 480;				// Fullscreen mode height.
static int prefDepth = 32;				// Fullscreen mode bit depth.
static int prefRefresh = 0;				// Fullscreen mode refresh rate. Set to 0 to use default rate.


// Image acquisition.
static ARUint8		*gARTImage = NULL;		//用于保存获取的一帧图片
static long		gCallCountMarkerDetect = 0;	//用于记录程序运行时到目前为止已经获取到的帧总数



// Markers.
ARMarkerNFT *markersNFT = NULL;				//（ARMarkerNFT数组）——用于保存nft标记的状态
int markersNFTCount = 0;				//保存在该程序中，表示有几个待追踪的nft目标



// NFT.
static THREAD_HANDLE_T     *threadHandle = NULL;	//在加载nft的时候进行设置（KPM线程开启时设置），保存线程的相关信息，作为trackingInitMain函数的参数
static AR2HandleT          *ar2Handle = NULL;		//保存nft texture追踪器的当前状态，主要是在traking过程中的一些设置信息
static KpmHandle           *kpmHandle = NULL;		//用于特征匹配函数kpmMatching()的结构体，保存了kpm tracking需要的参数（包括数据集）和kpm tracking的结果
static int                  surfaceSetCount = 0;	//保存本项目中nft标记的数量，在loadnft中设置
static AR2SurfaceSetT      *surfaceSet[PAGES_MAX];	//用于保存.fset数据集信息，每一个nft marker分开存放，用于AR2tracking




// Drawing.
static int gWindowW;
static int gWindowH;
static ARParamLT *gCparamLT = NULL;			//保存相机的内参
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;	//为OpenGL环境建立argl库后设置该参数
static int gDrawRotate = FALSE;
static float gDrawRotateAngle = 0;			// For use in drawing.
static ARdouble cameraLens[16];				//通过内参创建OpenGL投影矩阵时需要用到，作用未知



// ============================================================================
//	Function prototypes
// ============================================================================


static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p);
static int initNFT(ARParamLT *cparamLT, AR_PIXEL_FORMAT pixFormat);
static int loadNFTData(void);
static void cleanup(void);
static void Keyboard(unsigned char key, int x, int y);
static void Visibility(int visible);
static void Reshape(int w, int h);
static void Display(void);



// ============================================================================
//	Functions
// ============================================================================


int main(int argc, char** argv)
{
    char glutGamemode[32];
    const char *cparam_name = "Data2/camera_para.dat";			//用于保存相机参数文件路径，读取相机内参用
    char vconf[] = "";
    const char markerConfigDataFilename[] = "Data2/markers.dat";	//用于保存标记文件路径
	
    
#ifdef DEBUG
    arLogLevel = AR_LOG_LEVEL_DEBUG;
#endif


// ==============================ORB-SLAM2======================================
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.      //初始化所有线程
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,&ptr_State,true);  //字典文件、内参文件 SLAM是个system变量

    ptr_SLAM = &SLAM;

// =============================================================================


    //
	// Library inits.
	//
    
	glutInit(&argc, argv);		//初始化glut库
    
	//
	// Video setup.
	//
    
#ifdef _WIN32
	CoInitialize(NULL);
#endif
    
	if (!setupCamera(cparam_name, vconf, &gCparamLT)) {	//打开摄像头，读取相机内参，保存到gCparamLT中
		ARLOGe("main(): Unable to set up AR camera.\n");
		exit(-1);
	}
	
	
    //
    // AR init.
    //
    
    // Create the OpenGL projection from the calibrated camera parameters.
    arglCameraFrustumRH(&(gCparamLT->param), VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, cameraLens);	//根据相机内参构建OpenGL投影矩阵
    
    
    
    if (!initNFT(gCparamLT, arVideoGetPixelFormat())) {	//进行nft的初始化工作，主要是分配和初始化KPM tracking和ar2tracking需要用到的结构体kpmHandle、ar2Handle
		ARLOGe("main(): Unable to init NFT.\n");
		exit(-1);
    }

	//
	// Graphics setup.
	//
    
	// Set up GL context(s) for OpenGL to draw into.	//设置openGL初始化显示模式
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	
	if (!prefWindowed) {
		if (prefRefresh) sprintf(glutGamemode, "%ix%i:%i@%i", prefWidth, prefHeight, prefDepth, prefRefresh);
		else sprintf(glutGamemode, "%ix%i:%i", prefWidth, prefHeight, prefDepth);
		glutGameModeString(glutGamemode);
		glutEnterGameMode();
	} else {
		glutInitWindowSize(gCparamLT->param.xsize, gCparamLT->param.ysize);
		glutCreateWindow(argv[0]);
	}
    
	// Setup ARgsub_lite library for current OpenGL context.	//为当前OpenGL环境建立argl库
	if ((gArglSettings = arglSetupForCurrentContext(&(gCparamLT->param), arVideoGetPixelFormat())) == NULL) {
		ARLOGe("main(): arglSetupForCurrentContext() returned error.\n");
		cleanup();
		exit(-1);
	}
	
	arUtilTimerReset();	//在刚开始的时候即帧数为0的时候，复位定时器arUtilTimerReset，以便使用arUtilTimer得到从复位开始消耗的时间
    
    //
    // Markers setup.
    //
    
	
    // Load marker(s).
    newMarkers(markerConfigDataFilename, &markersNFT, &markersNFTCount);//加载marker文件信息，并设置markersNFT和markersNFTCount，其中markersNFT中的“valid validPrev ftmi filterCutoffFrequency filterSampleRate pageNo datasetPathname被设置”
    if (!markersNFTCount) {
        ARLOGe("Error loading markers from config. file '%s'.\n", markerConfigDataFilename);
		cleanup();
		exit(-1);
    }
    
    ARLOGi("Marker count = %d\n", markersNFTCount);
    
    
    // Marker data has been loaded, so now load NFT data.	//加载nft数据集（自然图像标记），设置kpmHandle->refDataSet
    if (!loadNFTData()) {
        ARLOGe("Error loading NFT data.\n");
		cleanup();
		exit(-1);
    }    
    
    // Start the video.
    if (arVideoCapStart() != 0) {				//打开摄像头开始捕捉视频帧
    	ARLOGe("setupCamera(): Unable to begin camera data capture.\n");
		return (FALSE);
	}
	
	// Register GLUT event-handling callbacks.
	// NB: mainLoop() is registered by Visibility.
	
	
	//设置glut的各个回调事件，然后使用glutMainLoop（）使得各个回调事件进行执行
	
	glutDisplayFunc(Display);		//当窗口需要重新绘制时，执行Display
	glutReshapeFunc(Reshape);		//当改变窗口大小时，执行Reshape
	glutVisibilityFunc(Visibility);		//当glut窗口得到显示时，执行visibility(第一次显示时，也会调用一次)
	
	glutKeyboardFunc(Keyboard);		//处理键盘输入的事件
	
	glutMainLoop();

	return (0);
}

// Something to look at, draw a rotating colour cube.
static void DrawCube(void)
{
    // Colour cube data.
    int i;
	float fSize = 40.0f;
    const GLfloat cube_vertices [8][3] = {
        /* +z */ {0.5f, 0.5f, 0.5f}, {0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f},
        /* -z */ {0.5f, 0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f} };
    const GLubyte cube_vertex_colors [8][4] = {
        {255, 255, 255, 255}, {255, 255, 0, 255}, {0, 255, 0, 255}, {0, 255, 255, 255},
        {255, 0, 255, 255}, {255, 0, 0, 255}, {0, 0, 0, 255}, {0, 0, 255, 255} };
    const GLubyte cube_faces [6][4] = { /* ccw-winding */
        /* +z */ {3, 2, 1, 0}, /* -y */ {2, 3, 7, 6}, /* +y */ {0, 1, 5, 4},
        /* -x */ {3, 0, 4, 7}, /* +x */ {1, 2, 6, 5}, /* -z */ {4, 5, 6, 7} };
    
    glPushMatrix(); // Save world coordinate system.
    glRotatef(gDrawRotateAngle, 0.0f, 0.0f, 1.0f); // Rotate about z axis.
    glScalef(fSize, fSize, fSize);
    glTranslatef(0.0f, 0.0f, 0.5f); // Place base of cube on marker surface.
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, cube_vertex_colors);
    glVertexPointer(3, GL_FLOAT, 0, cube_vertices);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    for (i = 0; i < 6; i++) {
        glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
    }
    glDisableClientState(GL_COLOR_ARRAY);
    glColor4ub(0, 0, 0, 255);
    for (i = 0; i < 6; i++) {
        glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
    }
    glPopMatrix();    // Restore world coordinate system.
}

static void DrawCubeUpdate(float timeDelta)
{
	if (gDrawRotate) {
		gDrawRotateAngle += timeDelta * 45.0f; // Rotate cube at 45 degrees per second.
		if (gDrawRotateAngle > 360.0f) gDrawRotateAngle -= 360.0f;
	}
}

static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p)
{	
    ARParam			cparam;				//用于保存相机内参
    int				xsize, ysize;			//保存视频窗口的大小(width、height)
    AR_PIXEL_FORMAT pixFormat;					//保存像素格式

    
    // Open the video path.					//打开相机
    if (arVideoOpen(vconf) < 0) {				//最后打开的相机是什么样的（包括视频窗口大小）与vconf有关
    	ARLOGe("setupCamera(): Unable to open connection to camera.\n");
    	return (FALSE);
	}
	
    // Find the size of the window.
    if (arVideoGetSize(&xsize, &ysize) < 0) {			//获取视频帧的大小（实际的视频窗口）
        ARLOGe("setupCamera(): Unable to determine camera frame size.\n");
        arVideoClose();
        return (FALSE);
    }
    ARLOGi("Camera image size (x,y) = (%d,%d)\n", xsize, ysize);
	
	// Get the format in which the camera is returning pixels.
	pixFormat = arVideoGetPixelFormat();			//获取视频帧的图片像素格式：AR_PIXEL_FORMAT_BGR
	if (pixFormat == AR_PIXEL_FORMAT_INVALID) {
    	ARLOGe("setupCamera(): Camera is using unsupported pixel format.\n");
        arVideoClose();
		return (FALSE);
	}
	
	    //ARLOGi("pixFormat: %d\n",pixFormat);		//经过测试，pixFormat=AR_PIXEL_FORMAT_BGR
	  
	// Load the camera parameters, resize for the window and init.
    if (arParamLoad(cparam_name, 1, &cparam) < 0) {		//读取相机内参（视频帧大小、投影矩阵P、镜头畸变参数），保存到cparam
		ARLOGe("setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
        arVideoClose();
        return (FALSE);
    }
    
    //测试相机内参：
    cout<<"矩阵："<<endl;
    for(int i=0;i<3;i++)
    {
      for(int j=0;j<4;j++)
	cout<<cparam.mat[i][j]<<"  ";
      cout<<endl;
    }
    
    cout<<"dist_factor:"<<endl;
    for(int i=0;i<AR_DIST_FACTOR_NUM_MAX;i++)
    cout<<cparam.dist_factor[i]<<endl;
    
    if (cparam.xsize != xsize || cparam.ysize != ysize) {	//修改cparam中的xsize、ysize为实际视频帧大小
        ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
        arParamChangeSize(&cparam, xsize, ysize, &cparam);
    }
    
#ifdef DEBUG
    ARLOG("*** Camera Parameter ***\n");
    arParamDisp(&cparam);
#endif
    
    //根据cparam信息构造了结构体ARParamLT对象，并使得全局变量gCparamLT（ARParamLT *）指向了它
    if ((*cparamLT_p = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {	
        ARLOGe("setupCamera(): Error: arParamLTCreate.\n");
        arVideoClose();
        return (FALSE);
    }
	
	return (TRUE);
}


// Modifies globals: kpmHandle, ar2Handle.

static int initNFT(ARParamLT *cparamLT, AR_PIXEL_FORMAT pixFormat)
{
    ARLOGd("Initialising NFT.\n");
    //
    // NFT init.
    //
    
    // KPM init.
    kpmHandle = kpmCreateHandle(cparamLT, pixFormat);	//为用于特征点的匹配函数kpmMatching()——KPM tracking 分配并初始化kpmHandle结构体,主要设置了其中的cparamLT、poseMode、xsize, ysize、pixFormat、procMode，暂未设置数据集信息
    if (!kpmHandle) {
        ARLOGe("Error: kpmCreateHandle.\n");
        return (FALSE);
    }
    //kpmSetProcMode( kpmHandle, KpmProcHalfSize );
    
    
    // AR2 init.
    if( (ar2Handle = ar2CreateHandle(cparamLT, pixFormat, AR2_TRACKING_DEFAULT_THREAD_NUM)) == NULL ) {
        ARLOGe("Error: ar2CreateHandle.\n");		//为ar2tracking初始化ar2Handle结构体，并创建多个ar2tracking线程等待处理
        kpmDeleteHandle(&kpmHandle);
        return (FALSE);
    }
    
    if (threadGetCPU() <= 1) {
        ARLOGi("Using NFT tracking settings for a single CPU.\n");
        ar2SetTrackingThresh(ar2Handle, 5.0);
        ar2SetSimThresh(ar2Handle, 0.50);
        ar2SetSearchFeatureNum(ar2Handle, 16);
        ar2SetSearchSize(ar2Handle, 6);
        ar2SetTemplateSize1(ar2Handle, 6);
        ar2SetTemplateSize2(ar2Handle, 6);
    } else {								//有4个tracking线程
        ARLOGi("Using NFT tracking settings for more than one CPU.\n");
        ar2SetTrackingThresh(ar2Handle, 5.0);			//设置位姿估计阈值
        ar2SetSimThresh(ar2Handle, 0.50);
        ar2SetSearchFeatureNum(ar2Handle, 16);
        ar2SetSearchSize(ar2Handle, 12);			//设置特征点搜索半径
        ar2SetTemplateSize1(ar2Handle, 6);
        ar2SetTemplateSize2(ar2Handle, 6);
    }
    // NFT dataset loading will happen later.
    return (TRUE);
}

// Modifies globals: threadHandle, surfaceSet[], surfaceSetCount
static int unloadNFTData(void)
{
    int i, j;
    
    if (threadHandle) {
        ARLOGi("Stopping NFT2 tracking thread.\n");
        trackingInitQuit(&threadHandle);
    }
    j = 0;
    for (i = 0; i < surfaceSetCount; i++) {
        if (j == 0) ARLOGi("Unloading NFT tracking surfaces.\n");
        ar2FreeSurfaceSet(&surfaceSet[i]); // Also sets surfaceSet[i] to NULL.
        j++;
    }
    if (j > 0) ARLOGi("Unloaded %d NFT tracking surfaces.\n", j);
    surfaceSetCount = 0;
    
    return 0;
}

// References globals: markersNFTCount
// Modifies globals: threadHandle, surfaceSet[], surfaceSetCount, markersNFT[]

static int loadNFTData(void)		//为ar2tracking 和 kpmtracking加载nft数据集，并开启kpm线程
{
    int i;
    KpmRefDataSet *refDataSet;
    
    // If data was already loaded, stop KPM tracking thread and unload previously loaded data.
    if (threadHandle) {
        ARLOGi("Reloading NFT data.\n");
        unloadNFTData();
    } else {	//该条语句执行
        ARLOGi("Loading NFT data.\n");
    }
    
    refDataSet = NULL;
    
    for (i = 0; i < markersNFTCount; i++) {
      
        // Load KPM data.
        KpmRefDataSet  *refDataSet2;
	
        ARLOGi("Reading %s.fset3\n", markersNFT[i].datasetPathname);	//读取文件pinball.fset3
	
        if (kpmLoadRefDataSet(markersNFT[i].datasetPathname, "fset3", &refDataSet2) < 0 ) {	//加载pinball.fset3中的数据到refDataSet2
            ARLOGe("Error reading KPM data from %s.fset3\n", markersNFT[i].datasetPathname);
            markersNFT[i].pageNo = -1;
            continue;
        }
        
        markersNFT[i].pageNo = surfaceSetCount;			//分别为每个marker进行编号
        
        ARLOGi("  Assigned page no. %d.\n", surfaceSetCount);
	
        if (kpmChangePageNoOfRefDataSet(refDataSet2, KpmChangePageNoAllPages, surfaceSetCount) < 0) {
            ARLOGe("Error: kpmChangePageNoOfRefDataSet\n");
            exit(-1);
        }
        if (kpmMergeRefDataSet(&refDataSet, &refDataSet2) < 0) {	//把数据集信息合并到refDataSet中并销毁refDataSet2
            ARLOGe("Error: kpmMergeRefDataSet\n");
            exit(-1);
        }
        ARLOGi("  Done.\n");
        
	
        // Load AR2 data.
        ARLOGi("Reading %s.fset\n", markersNFT[i].datasetPathname);	//读取文件pinball.fset
        
        if ((surfaceSet[surfaceSetCount] = ar2ReadSurfaceSet(markersNFT[i].datasetPathname, "fset", NULL)) == NULL ) {
            ARLOGe("Error reading data from %s.fset\n", markersNFT[i].datasetPathname);
        }					//把从pinball.fset中读出的信息放到surfaceSet中
        ARLOGi("  Done.\n");
        
        surfaceSetCount++;
        if (surfaceSetCount == PAGES_MAX) break;
    }
    
     
    //把数据集设置为kpm当前追踪的数据集
    if (kpmSetRefDataSet(kpmHandle, refDataSet) < 0) {	//将refDataSet更新到kpmHandle->refDataSet
        ARLOGe("Error: kpmSetRefDataSet\n");
        exit(-1);
    }
    kpmDeleteRefDataSet(&refDataSet);
    
    
    // Start the KPM tracking thread.
    threadHandle = trackingInitInit(kpmHandle);		//开启KPM线程
    if (!threadHandle) exit(-1);

	ARLOGi("Loading of NFT data complete.\n");
    return (TRUE);
}

static void cleanup(void)
{
    if (markersNFT) deleteMarkers(&markersNFT, &markersNFTCount);
    
    // NFT cleanup.
    unloadNFTData();
    ARLOGd("Cleaning up ARToolKit NFT handles.\n");
    ar2DeleteHandle(&ar2Handle);
    kpmDeleteHandle(&kpmHandle);
    arParamLTFree(&gCparamLT);

    // OpenGL cleanup.
    arglCleanup(gArglSettings);
    gArglSettings = NULL;
    
    // Camera cleanup.
	arVideoCapStop();
	arVideoClose();
#ifdef _WIN32
	CoUninitialize();
#endif
}

static void Keyboard(unsigned char key, int x, int y)
{
	ARLOGi("keyboard\n");
	switch (key) {
		case 0x1B:						// Quit.
		case 'Q':
		case 'q':
			cleanup();
			exit(0);
			break;
		case ' ':
			gDrawRotate = !gDrawRotate;
			break;
		case '?':
		case '/':
			ARLOG("Keys:\n");
			ARLOG(" q or [esc]    Quit demo.\n");
			ARLOG(" ? or /        Show this help.\n");
			ARLOG("\nAdditionally, the ARVideo library supplied the following help text:\n");
			arVideoDispOption();
			break;
		default:
			break;
	}
}

static void mainLoop(void)
{
	static int ms_prev;
	int ms;
	float s_elapsed;
	ARUint8 *image;

    // NFT results.
    static int detectedPage = -2; // -2 Tracking not inited, -1 tracking inited OK, >= 0 tracking online on page.
    static float trackingTrans[3][4];	//用于存放模板坐标系到相机坐标系的变换矩阵
    

    int             i, j, k;


	// Find out how long since mainLoop() last ran.		//计算两次mainloop的执行间隔
	ms = glutGet(GLUT_ELAPSED_TIME);
	s_elapsed = (float)(ms - ms_prev) * 0.001f;
	if (s_elapsed < 0.01f) return; // Don't update more often than 100 Hz.	//如果两次时间<0.01s,则返回（使得刷新频率低于100hz）
	ms_prev = ms;
	
	
	// Update drawing.		
	DrawCubeUpdate(s_elapsed);		//当在键盘上输入space后，该条语句会执行
	
	
	// Grab a video frame.
	if ((image = arVideoGetImage()) != NULL) {	//调用函数获取一帧图片
		gARTImage = image;	// Save the fetched image.
		gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.

        // Run marker detection on frame
        if (threadHandle) {	//kpm的threadhandle
            // Perform NFT tracking.
            float            err;
            int              ret;
            int              pageNo;
            
            if( detectedPage == -2 ) {		//把新获取到的帧传递给threadHandle->trackingInitHandle->imageSize，并向kpmtracking发送start信号
                trackingInitStart( threadHandle, gARTImage );
                detectedPage = -1;		//设置detectedPage = -1表示图片已经传递给kpmtracking,且kpmtracking已经开始工作
            }
            
            if( detectedPage == -1 ) {		//kpmtracking已经开始工作
	      
                ret = trackingInitGetResult( threadHandle, trackingTrans, &pageNo);	//尝试获取kpmtracking的结果——即相机的位姿，如果成功，则返回1，否则返回-1
		
                if( ret == 1 ) {	//成功从kpmtracking获取到相机位姿
                    if (pageNo >= 0 && pageNo < surfaceSetCount) {//如果返回的相机位姿正确，则把后续获取位姿的工作从kpmtracking转交到ar2tracking
                        ARLOGi("Detected page %d.\n", pageNo);
			
                        detectedPage = pageNo;			//设置detectedPage以进行后续的ar2Tracking
                        
                        ar2SetInitTrans(surfaceSet[detectedPage], trackingTrans);//把从kpmtracking处得到的位姿结果放入surfaceSet->trans1，并设置surfaceSet->contNum=1，surfaceSet->prevFeature[0].flag=-1
                    } else {
                        ARLOGi("Detected bad page %d.\n", pageNo);
                        detectedPage = -2;
                    }
                } else if( ret < 0 ) {		//未检测到位姿
                    ARLOGi("No page detected.\n");
                    detectedPage = -2;
                }
            }
            
	     //检测模板成功后每次都直接从这里开始
            if( detectedPage >= 0 && detectedPage < surfaceSetCount) {	//当kpmtracking检测到Marker并得到位姿后，使用该值进行初始化ar2tracking并交由ar2tracking获取后续的位姿
                if( ar2Tracking(ar2Handle, surfaceSet[detectedPage], gARTImage, trackingTrans, &err) < 0 ) {	//该函数的作用类似于KPMtracking
                    ARLOGd("Tracking lost.\n");								//跟踪丢失
                    detectedPage = -2;
                } else {
                    ARLOGi("Tracked page %d (max %d).\n", detectedPage, surfaceSetCount - 1);		//跟踪成功
		    	
                }
            }
            
        } else {
            ARLOGe("Error: threadHandle\n");
            detectedPage = -2;
        }
        
        // Update markers.
        for (i = 0; i < markersNFTCount; i++) {
	  
            markersNFT[i].validPrev = markersNFT[i].valid;
	    
            if (markersNFT[i].pageNo >= 0 && markersNFT[i].pageNo == detectedPage) {	//ar2tracking跟踪成功
                markersNFT[i].valid = TRUE;
		
                for (j = 0; j < 3; j++) for (k = 0; k < 4; k++) markersNFT[i].trans[j][k] = trackingTrans[j][k];	//如果ar2tracking跟踪成功，则把makersNFT[i].trans设置为ar2tracking的结果
            
	      
	    }
            else markersNFT[i].valid = FALSE;		//这次跟踪失败
	    
	    
            if (markersNFT[i].valid) {		//这次成功
                
		
                // Filter the pose estimate.
		
                if (markersNFT[i].ftmi) {
                    if (arFilterTransMat(markersNFT[i].ftmi, markersNFT[i].trans, !markersNFT[i].validPrev) < 0) {	//对位姿结果进行滤波优化
                        ARLOGe("arFilterTransMat error with marker %d.\n", i);
                    }
                }
 
		
                // ====================ORB-SLAM2============================

		ARfloat   trans_temp[4][4];
		for(int m=0;m<3;m++)
		  for(int n=0;n<4;n++)
		    trans_temp[m][n]=markersNFT[i].trans[m][n];
		  
		trans_temp[3][0]=trans_temp[3][1]=trans_temp[3][2]=0;
		trans_temp[3][3]=1;    
		cv::Mat Tcm(4,4,CV_32F,(ARfloat *)trans_temp);	//得到模板坐标系到相机坐标系的变换矩阵		
		
		cv::Mat im(480,640,CV_8UC3,gARTImage);	//构造mat类型图像
	
		
		if(im.empty())
		{
		  cerr << endl << "Failed to load image from ARToolkit."<<endl;
		  return;
		}
		
		//获取当前的时间戳
		std::chrono::system_clock::time_point t_epoch;
		std::chrono::duration<double,std::ratio<1,1000000>> duration_micr_sec = std::chrono::system_clock::now() - t_epoch;
		double tframe = duration_micr_sec.count() / 1000000.0;
		
		(*ptr_SLAM).TrackMonocular(im,tframe,Tcm,1); //将图片和时间戳以及位姿传入SLAM系统
		
                // =========================================================
                
                if (!markersNFT[i].validPrev) {		//上次失败？——marker从未识别到识别
                    // Marker has become visible, tell any dependent objects.
                    // --->
                }
                
                // We have a new pose, so set that.
                arglCameraViewRH((const ARdouble (*)[4])markersNFT[i].trans, markersNFT[i].pose.T, VIEW_SCALEFACTOR);
                // Tell any dependent objects about the update.
                // --->
                
            } else {			//这次失败，在ARToolkit追踪失败的情况下尝试从ORB-SLAM获取计算得到的位姿
                
		if((*ptr_State)==ORB_SLAM2::System::OK || (*ptr_State)==ORB_SLAM2::System::LOST)//若ORB-SLAM初始化成功
		{
		    cv::Mat im(480,640,CV_8UC3,gARTImage);	//构造mat类型图像
		    
		    if(im.empty())
		    {
		      cerr << endl << "Failed to load image from ARToolkit."<<endl;
		      return;
		    }
		    
		    //获取当前的时间戳
		    std::chrono::system_clock::time_point t_epoch;
		    std::chrono::duration<double,std::ratio<1,1000000>> duration_micr_sec = std::chrono::system_clock::now() - t_epoch;
		    double tframe = duration_micr_sec.count() / 1000000.0;	
		    
		    cv::Mat Tcm((*ptr_SLAM).TrackMonocular(im,tframe,0)); //将图片和时间戳传入SLAM系统,得到当前模板坐标系到相机坐标系的变换矩阵
		    
		    if((*ptr_State)==ORB_SLAM2::System::OK)	//位姿有效(SLAM跟踪未丢失)
		    {
		      cout<<"跟踪未丢失Tcm:"<<endl<<Tcm<<endl;
		      for(int m=0;m<3;m++)
			for(int n=0;n<4;n++)
			  markersNFT[i].trans[m][n]=(ARdouble)Tcm.at<float>(m,n);
			
		      markersNFT[i].valid = TRUE;	//更改本次的跟踪状态为成功
		      
		      // Filter the pose estimate.
		      if (markersNFT[i].ftmi) {		//对位姿结果进行滤波优化
			if (arFilterTransMat(markersNFT[i].ftmi, markersNFT[i].trans, 0) < 0) {
                        ARLOGe("arFilterTransMat error with marker %d.\n", i);
			}
		     }		      

		      // We have a new pose, so set that.
		      arglCameraViewRH((const ARdouble (*)[4])markersNFT[i].trans, markersNFT[i].pose.T, VIEW_SCALEFACTOR);
		    }
		}
		
                if (markersNFT[i].validPrev) {	//上次成功？
                    // Marker has ceased to be visible, tell any dependent objects.	//marker从识别到丢失
                    // --->
                }
            }  
            
            
        }
		// Tell GLUT the display has changed.
		glutPostRedisplay();			//告诉display()进行显示
	}
}

//
//	This function is called on events when the visibility of the
//	GLUT window changes (including when it first becomes visible).
//
static void Visibility(int visible)
{
	if (visible == GLUT_VISIBLE) {
		glutIdleFunc(mainLoop);		//经测试mainloop会一直不断得到执行
	} else {
		glutIdleFunc(NULL);
	}
}

//
//	This function is called when the
//	GLUT window is resized.
//
static void Reshape(int w, int h)
{
    gWindowW = w;
    gWindowH = h;
    
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	
	// Call through to anyone else who needs to know about window sizing here.
}

//
// This function is called when the window needs redrawing.
//
static void Display(void)
{
    //static int test = 0;
    int i;
    
	// Select correct buffer for this context.
	glDrawBuffer(GL_BACK);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.
    
    arglPixelBufferDataUpload(gArglSettings, gARTImage);	//这次捕获到的新帧
	arglDispImage(gArglSettings);
	
	gARTImage = NULL; // Invalidate image data.		//设置完成后无效图像指针
				
    // Set up 3D mode.
	glMatrixMode(GL_PROJECTION);
#ifdef ARDOUBLE_IS_FLOAT
	glLoadMatrixf(cameraLens);
#else
	glLoadMatrixd(cameraLens);
#endif
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    glEnable(GL_DEPTH_TEST);

    // Set any initial per-frame GL state you require here.
    // --->
    
    // Lighting and geometry that moves with the camera should be added here.
    // (I.e. should be specified before marker pose transform.)
    // --->
    
    for (i = 0; i < markersNFTCount; i++) {
        
        if (markersNFT[i].valid) {			//当前的marker跟踪有效才显示
        
#ifdef ARDOUBLE_IS_FLOAT
            glLoadMatrixf(markersNFT[i].pose.T);
#else
            glLoadMatrixd(markersNFT[i].pose.T);	//display最后使用的是markersNFT[i].pose.T
#endif
	    
            // All lighting and geometry to be drawn relative to the marker goes here.
            // --->
            DrawCube();
        }
    }
    
    // Set up 2D mode.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, (GLdouble)gWindowW, 0, (GLdouble)gWindowH, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    // Add your own 2D overlays here.
    // --->
    
	glutSwapBuffers();
}
