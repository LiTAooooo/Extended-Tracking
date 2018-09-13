/* 
 *  ARMarkerNFT.h
 *  ARToolKit5
 *
 *  Disclaimer: IMPORTANT:  This Daqri software is supplied to you by Daqri
 *  LLC ("Daqri") in consideration of your agreement to the following
 *  terms, and your use, installation, modification or redistribution of
 *  this Daqri software constitutes acceptance of these terms.  If you do
 *  not agree with these terms, please do not use, install, modify or
 *  redistribute this Daqri software.
 *
 *  In consideration of your agreement to abide by the following terms, and
 *  subject to these terms, Daqri grants you a personal, non-exclusive
 *  license, under Daqri's copyrights in this original Daqri software (the
 *  "Daqri Software"), to use, reproduce, modify and redistribute the Daqri
 *  Software, with or without modifications, in source and/or binary forms;
 *  provided that if you redistribute the Daqri Software in its entirety and
 *  without modifications, you must retain this notice and the following
 *  text and disclaimers in all such redistributions of the Daqri Software.
 *  Neither the name, trademarks, service marks or logos of Daqri LLC may
 *  be used to endorse or promote products derived from the Daqri Software
 *  without specific prior written permission from Daqri.  Except as
 *  expressly stated in this notice, no other rights or licenses, express or
 *  implied, are granted by Daqri herein, including but not limited to any
 *  patent rights that may be infringed by your derivative works or by other
 *  works in which the Daqri Software may be incorporated.
 *
 *  The Daqri Software is provided by Daqri on an "AS IS" basis.  DAQRI
 *  MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION
 *  THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE, REGARDING THE DAQRI SOFTWARE OR ITS USE AND
 *  OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS.
 *
 *  IN NO EVENT SHALL DAQRI BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL
 *  OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION,
 *  MODIFICATION AND/OR DISTRIBUTION OF THE DAQRI SOFTWARE, HOWEVER CAUSED
 *  AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
 *  STRICT LIABILITY OR OTHERWISE, EVEN IF DAQRI HAS BEEN ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Copyright 2015 Daqri LLC. All Rights Reserved.
 *  Copyright 2013-2015 ARToolworks, Inc. All Rights Reserved.
 *
 *  Author(s): Philip Lamb.
 *
 */

#ifndef __ARMarkerNFT_h__
#define __ARMarkerNFT_h__

#include <AR/ar.h>
#include <AR/arFilterTransMat.h>
#ifndef _MSC_VER
#  include <stdbool.h>
#else
typedef unsigned char bool;
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ARdouble v[3];
} ARVec3;

typedef struct {
    ARdouble T[16]; // Position and orientation, column-major order. (position(x,y,z) = {T[12], T[13], T[14]}
} ARPose;

extern const ARPose ARPoseUnity;

typedef struct _ARMarkerNFT {
    // ARMarker protected 
    bool       valid;			//在加载marker的时候被初始化为false
    bool       validPrev;		//在加载marker的时候被初始化为false
    ARdouble   trans[3][4];		//保存相机当前的位姿
    ARPose     pose;			//保存marker的位姿
    ARdouble   marker_width;		//保存marker的宽度
    ARdouble   marker_height;		//保存marker的高度
    
    // ARMarker private
    ARFilterTransMatInfo *ftmi;		//该结构体用于对marker的位姿进行滤波（优化），通过filterCutoffFrequency和filterSampleRate初始化而来
    ARdouble   filterCutoffFrequency;	//在marker.dat中的FILTER处设置，为15
    ARdouble   filterSampleRate;	//在加载marker时被设置为默认值
    
    // ARMarkerNFT
    int        pageNo;			//在加载marker的时候被初始化为-1,在加载nft数据集的时候开始从0编号(用于表示当前nftmarker是几号图片)
    char      *datasetPathname;		//用于保存NFT数据集的路径名
} ARMarkerNFT;			//每个该结构体的对象代表一个自然图像的marker

void newMarkers(const char *markersConfigDataFilePathC, ARMarkerNFT **markersNFT_out, int *markersNFTCount_out);

void deleteMarkers(ARMarkerNFT **markersNFT_p, int *markersNFTCount_p);

#ifdef __cplusplus
}
#endif
#endif // !__ARMarkerNFT_h__