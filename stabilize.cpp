/*
    DePan plugin for Avisynth 2.5 - global motion compensation
	Version 1.10.0, May 6, 2007.
	(DePanStabilize function)
	Copyright(c)2004-2007, A.G. Balakhnin aka Fizick
	bag@hotmail.ru

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

		DePan plugin at first stage calculates global motion in frames (by phase-shift method),
	and at second stage shift frame images for global motion compensation

		DePanStabilize function make some motion stabilization (deshake)
	by inertial filtering of global motion.

		Inertial motion filtering method is partially based on article:
	Camera Stabilization Based on 2.5D Motion Estimation and Inertial Motion Filtering.
	by Zhigang Zhu, Guangyou Xu, Yudong Yang, Jesse S. Jin,
	1998 IEEE International Conference on Intelligent Vehicles.
	(File: zhu98camera.pdf)

	Parameters of DePanStabilize:
		clip - input clip (the same as input clip in DePanEstimate)
		data - special service clip with coded motion data, produced by DePanEstimate
		cutoff - vibration frequency cutoff, Hertz
		damping - relative damping coefficient
		addzoom - add adaptive zoom
		prev - previous frame lag to fill border
		next - next frame lag to fill border
		mirror - use mirror to fill border
		dxmax - limit of horizontal correction, in pixels
		dymax - limit of vertical correction, in pixels
		zoommax - limit of zoom correction ( smooth only adaptive zoom)
		rotmax - limit of rotation correction, in degrees
			(if any above max parameter is positive, the correction will be limited to it,
			if negative, the correction will be null and frame set as new base )
		subpixel - pixel interpolation accuracy
		pixaspect - pixel aspect
		fitlast - fit some last frames range to original position
		tzoom - adaptive zoom rise time, sec
		initzoom - initial zoom
		info - show motion info on frame
		inputlog - name of input log file in Deshaker format (default - none, not read)
		method - stablilization method number (0-inertial, 1-average)

*/

#include <algorithm>

#include <windowsPorts/windows2linux.h>
#include <avxplugin.h>
#include "math.h"
#include "float.h"
//#include "stdio.h"

#include "info.h"
#include "depanio.h"
#include "depan.h"


//****************************************************************************
class DePanStabilize : public GenericVideoFilter {
  // DePanStabilize defines the name of your filter class.
  // This name is only used internally, and does not affect the name of your filter or similar.
  // This filter extends GenericVideoFilter, which incorporates basic functionality.
  // All functions present in the filter must also be present here.

	PClip DePanData;      // motion data clip
	float cutoff; // vibration frequency cutoff
	float damping; // relative damping coeff
	float initzoom; // initial zoom - added in v.1.7
	bool addzoom; // add adaptive zoom
	int fillprev;
	int fillnext;
	int mirror;
	int blur; // mirror blur len - v.1.3
	float dxmax;  // deshake limit of dx
	float dymax;  // deshake limit of dy
	float zoommax;
	float rotmax; // degrees
	int subpixel;     // subpixel accuracy by interpolation
	float pixaspect; // pixel aspect
	int fitlast; // range of last frame to fit to original position - added in v.1.2
	float tzoom; // adaptive zoom rise time, sec - added in v. 1.5
	int info;   // show info on frame
	const char *inputlog;  // filename of input log file in Deshaker format
	const char *vdx; // global parameter dx name
	const char *vdy; // global parameter dy name
	const char *vzoom; // global parameter zoom name
	const char *vrot; // global parameter rot name
	int method; // stabllization method


	// internal parameters
//	int matchfields;
//	int fieldbased;
	int nfields;
	int TFF;
	float fps; // frame per second
	float mass; // mass
	float pdamp;  // damping parameter
	float kstiff; // stiffness
	float freqnative;  // native frequency
//	float s1,s2,c0,c1,c2,cnl; // smoothing filter coefficients
	int nbase; // base frame for stabilization
	int radius; // stabilization radius

// motion tables
	float * motionx;
	float * motiony;
	float * motionzoom;
	float * motionrot;

	int * work2width4356;  // work array for interpolation

	transform * trcumul; // cumulative transforms array
	transform * trsmoothed; // smoothed cumulative transforms array

	transform nonlinfactor; // it is not transform, but approximate inverse limit values for components

	float * azoom; // adaptive zooms
	float * azoomsmoothed; // adaptive zooms smoothed

	// planes from YUY2 - v1.6
	BYTE * srcplaneY;
	BYTE * srcplaneU;
	BYTE * srcplaneV;
	BYTE * dstplaneY;
	BYTE * dstplaneU;
	BYTE * dstplaneV;
	int planeYpitch;
	int planeUVpitch;
	int planeYwidth, planeUVwidth;

	float vdx_val, vdy_val, vzoom_val, vrot_val; // AviSynth permanent variables (SetVar)

	float * wint; // average window
	int wintsize;
	float * winrz; // rize zoom window
	float * winfz; // fall zoom window
	int winrzsize;
	int winfzsize;

	float xcenter;  // center of frame
	float ycenter;

	void Inertial(int _nbase, int _ndest, transform * trdif);
	void Average(int nbase, int ndest, int nmax, transform * trdif);
	void InertialLimit(float *dxdif, float *dydif, float *zoomdif, float *rotdif, int ndest, int *nbase);
	float Averagefraction(float dxdif, float dydif, float zoomdif, float rotdif);
public:
  // This defines that these functions are present in your class.
  // These functions must be that same as those actually implemented.
  // Since the functions are "public" they are accessible to other classes.
  // Otherwise they can only be called from functions within the class itself.

	DePanStabilize(PClip _child, PClip _DePanData, float _cutoff, float _damping,
		float _initzoom, bool _addzoom, int _fillprev, int _fillnext, int _mirror, int _blur, float _dxmax,
		float _dymax, float _zoommax, float _rotmax, int _subpixel, float _pixaspect,
		int _fitlast, float _tzoom, int _info, const char * _inputlog,
		const char * _vdx, const char * _vdy, const char * _vzoom, const char * _vrot, int _method, IScriptEnvironment* env);
  // This is the constructor. It does not return any value, and is always used,
  //  when an instance of the class is created.
  // Since there is no code in this, this is the definition.

  ~DePanStabilize();
  // The is the destructor definition. This is called when the filter is destroyed.


	PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
  // This is the function that AviSynth calls to get a given frame.
  // So when this functions gets called, the filter is supposed to return frame n.

};

//***************************
//  The following is the implementation
//  of the defined functions.
// ***************************
//****************************************************************************

//Here is the actual constructor code used
DePanStabilize::DePanStabilize(PClip _child, PClip _DePanData, float _cutoff,
	float _damping,  float _initzoom, bool _addzoom, int _fillprev, int _fillnext,
	int _mirror, int _blur, float _dxmax, float _dymax, float _zoommax,
	float _rotmax, int _subpixel, float _pixaspect,
	int _fitlast, float _tzoom, int _info, const char * _inputlog,
	const char * _vdx, const char * _vdy, const char * _vzoom, const char * _vrot, int _method, IScriptEnvironment* env) :

	GenericVideoFilter(_child), DePanData(_DePanData), cutoff(_cutoff), damping(_damping),
		initzoom(_initzoom), addzoom(_addzoom), fillprev(_fillprev), fillnext(_fillnext), mirror(_mirror), blur(_blur),
		dxmax(_dxmax), dymax(_dymax), zoommax(_zoommax), rotmax(_rotmax), subpixel(_subpixel),
		pixaspect(_pixaspect), fitlast(_fitlast), tzoom(_tzoom),  info(_info), inputlog(_inputlog),
	vdx(_vdx), vdy(_vdy), vzoom(_vzoom), vrot(_vrot), method(_method){
  // This is the implementation of the constructor.
  // The child clip (source clip) is inherited by the GenericVideoFilter,
  //  where the following variables gets defined:
  //   PClip child;   // Contains the source clip.
  //   VideoInfo vi;  // Contains videoinfo on the source clip.

//	char debugbuf[100];
	int error;
	int loginterlaced;
//	int nfields;
	float lambda;

//	matchfields =1;

//	zoommax = std::max(zoommax, initzoom); // v1.7 to prevent user error
	zoommax = zoommax > 0 ? std::max(zoommax, initzoom) : -std::max(-zoommax, initzoom) ; // v1.8.2

	int fieldbased = (vi.IsFieldBased()) ? 1 : 0;
	TFF = (vi.IsTFF() ) ? 1 : 0;
	// correction for fieldbased
	if (fieldbased != 0) 	nfields = 2;
	else nfields = 1;

	if (!vi.IsYV12() && !vi.IsYUY2())
		env->ThrowError("DePanStabilize: input must be YV12 or YUY2!");//v1.6

	if ( (DePanData->GetVideoInfo().num_frames) != vi.num_frames )
		env->ThrowError("DePanStabilize: The length of input clip must be same as motion data clip !");

	motionx = (float *)malloc(vi.num_frames*sizeof(float));
	motiony = (float *)malloc(vi.num_frames*sizeof(float));
	motionrot = (float *)malloc(vi.num_frames*sizeof(float));
	motionzoom = (float *)malloc(vi.num_frames*sizeof(float));

	work2width4356 = (int *)malloc((2*vi.width+4356)*sizeof(int)); // work


	trcumul = (transform *)malloc(vi.num_frames*sizeof(transform));
	trsmoothed = (transform *)malloc(vi.num_frames*sizeof(transform));

	azoom = (float *)malloc(vi.num_frames*sizeof(float));
	azoomsmoothed = (float *)malloc(vi.num_frames*sizeof(float));

//	child->SetCacheHints(CACHE_RANGE,3);
//	DePanData->SetCacheHints(CACHE_RANGE,3);

	if (inputlog != "") { // motion data will be readed from deshaker.log file once at start
		error = read_deshakerlog(inputlog,vi.num_frames,motionx,motiony,motionrot,motionzoom,&loginterlaced);
		if (error==-1)	env->ThrowError("DePan: Input log file not found!");
		if (error==-2)	env->ThrowError("DePan: Error input log file format!");
		if (error==-3)	env->ThrowError("DePan: Too many frames in input log file!");
//		if(vi.IsFieldBased  && loginterlaced==0)	env->ThrowError("DePan: Input log must be in interlaced for fieldbased!");
	}
	else { // motion data will be requesred from DepanEstimate
		if ( (DePanData->GetVideoInfo().num_frames) != child->GetVideoInfo().num_frames )
			env->ThrowError("DePan: The length of input clip must be same as motion data clip  !");

		for (int i=0; i<(vi.num_frames); i++) {
			motionx [i] = MOTIONUNKNOWN;  // init as unknown for all frames
//			motionrot[i] = 0;  // zero rotation for all frames (it is not estimated in current version)
		}
	}

	// prepare coefficients for inertial motion smoothing filter

		// termination (max) frequency for vibration eliminating in Hertz
		// damping ratio
//		damping = 0.9; // unfixed in v.1.1.4 (was accidentally fixed =0.9 in all previous versions  :-)
		// elastic stiffness of spring
		kstiff = 1.0;  // value is not important - (not included in result)
		//  relative frequency lambda at half height of response
		lambda = sqrtf( 1+6*damping*damping + sqrtf((1+6*damping*damping)*(1+6*damping*damping)+3) );
		// native oscillation frequency
		freqnative = cutoff/lambda;
		// mass of camera
		mass = kstiff/( (6.28f*freqnative)*(6.28f*freqnative) );
		// damping parameter
//		pdamp = 2*damping*sqrtf(mass*kstiff);
//		pdamp = 2*damping*sqrtf(kstiff*kstiff/( (6.28f*freqnative)*(6.28f*freqnative) ));
		pdamp = 2*damping*kstiff/(6.28f*freqnative);
		// frames per secomd
		fps = float(vi.fps_numerator)/vi.fps_denominator;

		// old smoothing filter coefficients from paper
//		float a1 = (2*mass + pdamp*period)/(mass + pdamp*period + kstiff*period*period);
//		float a2 = -mass/(mass + pdamp*period + kstiff*period*period);
//		float b1 = (pdamp*period + kstiff*period*period)/(mass + pdamp*period + kstiff*period*period);
//		float b2 = -pdamp*period/(mass + pdamp*period + kstiff*period*period);

/*		s1 = (2*mass*fps*fps - kstiff)/(mass*fps*fps + pdamp*fps/2);
		s2 = (-mass*fps*fps + pdamp*fps/2)/(mass*fps*fps + pdamp*fps/2);
		c0 = pdamp*fps/2/(mass*fps*fps + pdamp*fps/2);
		c1 = kstiff/(mass*fps*fps + pdamp*fps/2);
		c2 = -pdamp*fps/2/(mass*fps*fps + pdamp*fps/2);
		cnl = -kstiff/(mass*fps*fps + pdamp*fps/2); // nonlinear
*/
		// approximate factor values for nonlinear members as half of max
		if (dxmax != 0) { // chanded in v1.7
			nonlinfactor.dxc = 5/fabsf(dxmax);// chanded in v1.7
		}
		else {
			nonlinfactor.dxc = 0;
		}
		if (fabsf(zoommax) != 1) {// chanded in v1.7
			nonlinfactor.dxx = 5/(fabsf(zoommax)-1);// chanded in v1.7
			nonlinfactor.dyy = 5/(fabsf(zoommax)-1);// chanded in v1.7
		}
		else {
			nonlinfactor.dxx = 0;
			nonlinfactor.dyy = 0;
		}
		if (dymax != 0) { // chanded in v1.7
			nonlinfactor.dyc = 5/fabsf(dymax);// chanded in v1.7
		}
		else {
			nonlinfactor.dyc = 0;
		}
		if (rotmax != 0) {// chanded in v1.7
			nonlinfactor.dxy = 5/fabsf(rotmax);// chanded in v1.7
			nonlinfactor.dyx = 5/fabsf(rotmax);// chanded in v1.7
		}
		else {
			nonlinfactor.dxy = 0;
			nonlinfactor.dyx = 0;
		}

		nbase = 0;

		// get smoothed cumulative transform members
//		sprintf(debugbuf,"DePanStabilize: freqnative=%f mass=%f damp=%f\n", freqnative, mass, damp);
//		OutputDebugString(debugbuf);

	if (vi.IsYUY2()) // v1.6
	{ // create planes from YUY2
		planeYwidth=vi.width;
		planeUVwidth=vi.width/2;
		planeYpitch = ((vi.width+15)/16)*16;
		planeUVpitch = ((vi.width/2+15)/16)*16;
		srcplaneY = (BYTE*) malloc(planeYpitch*vi.height);
		srcplaneU = (BYTE*) malloc(planeUVpitch*vi.height);
		srcplaneV = (BYTE*) malloc(planeUVpitch*vi.height);
		dstplaneY = (BYTE*) malloc(planeYpitch*vi.height);
		dstplaneU = (BYTE*) malloc(planeUVpitch*vi.height);
		dstplaneV = (BYTE*) malloc(planeUVpitch*vi.height);
	}

	initzoom = 1/initzoom; // make consistent with internal definition - v1.7

	wintsize = int(fps/(4*cutoff));
	radius  = wintsize;
	wint = (float *)malloc((wintsize+1)*sizeof(float));

	float PI = 3.14159265258f;
	for (int i=0; i<wintsize; i++)
		wint[i] = cosf(i*0.5f*PI/wintsize);
	wint[wintsize] = 0;

	winrz = (float *)malloc((wintsize+1)*sizeof(float));
	winfz = (float *)malloc((wintsize+1)*sizeof(float));
	winrzsize = std::min(wintsize,int(fps*tzoom/4));
//	winfzsize = std::min(wintsize,int(fps*tzoom*1.5/4));
	winfzsize = std::min(wintsize,int(fps*tzoom/4));
	for (int i=0; i<winrzsize; i++)
		winrz[i] = cosf(i*0.5f*PI/winrzsize);
	for (int i=winrzsize; i<=wintsize; i++)
		winrz[i] = 0;
	for (int i=0; i<winfzsize; i++)
		winfz[i] = cosf(i*0.5f*PI/winfzsize);
	for (int i=winfzsize; i<=wintsize; i++)
		winfz[i] = 0;

	xcenter = vi.width/2.0f;  // center of frame
	ycenter = vi.height/2.0f;

}

//****************************************************************************
// This is where any actual destructor code used goes
DePanStabilize::~DePanStabilize() {
  // This is where you can deallocate any memory you might have used.
	free(motionx);
	free(motiony);
	free(motionzoom);
	free(motionrot);
	free(work2width4356);

	free(trcumul);
	free(trsmoothed);
	free(azoom); // bug fixed with memory leakage in v.0.9.1
	free(azoomsmoothed); // bug fixed with memory leakage in v.0.9.1
	if (vi.IsYUY2()) // v1.6
	{
		free(srcplaneY);
		free(srcplaneU);
		free(srcplaneV);
		free(dstplaneY);
		free(dstplaneU);
		free(dstplaneV);
	}
	free(wint);
	free(winrz);
	free(winfz);

}


//

void DePanStabilize::Inertial(int nbase, int ndest, transform * ptrdif)
{
	int n;
	transform trnull, trinv, trcur, trtemp;
	float zoom = 1; // make null transform
	motion2transform (0, 0, 0, zoom, 1, 0, 0, 1, 1.0, &trnull);

		// first 2 frames near base not smoothed, simple copy (sum with null)
		sumtransform(trcumul[nbase],trnull,&trsmoothed[nbase]); // copy cumul to smoothed for base
		sumtransform(trcumul[nbase+1],trnull,&trsmoothed[nbase+1]); // copy cumul to smoothed for base+1

		float cdamp = 12.56f*damping/fps;
		float cquad = 39.44f/(fps*fps);

		// recurrent calculation of smoothed cumulative transforms from base+2 to ndest frames
		for (n=nbase+2; n<=ndest; n++) {

			// dxc predictor:
			trsmoothed[n].dxc = 2*trsmoothed[n-1].dxc - trsmoothed[n-2].dxc - \
				cdamp*freqnative*( trsmoothed[n-1].dxc - trsmoothed[n-2].dxc - trcumul[n-1].dxc + trcumul[n-2].dxc )* \
				( 1 + 0.5f*nonlinfactor.dxc/freqnative*fabsf(trsmoothed[n-1].dxc - trsmoothed[n-2].dxc - trcumul[n-1].dxc + trcumul[n-2].dxc) ) - \
				cquad*freqnative*freqnative*(trsmoothed[n-1].dxc - trcumul[n-1].dxc)* \
				( 1 + nonlinfactor.dxc*fabsf(trsmoothed[n-1].dxc - trcumul[n-1].dxc) );    // predictor
			// corrector, one iteration must be enough:
			trsmoothed[n].dxc = 2*trsmoothed[n-1].dxc - trsmoothed[n-2].dxc - \
				cdamp*freqnative*0.5f*( trsmoothed[n].dxc - trsmoothed[n-2].dxc - trcumul[n].dxc + trcumul[n-2].dxc )* \
				( 1 + 0.5f*nonlinfactor.dxc/freqnative*0.5f*fabsf(trsmoothed[n].dxc - trsmoothed[n-2].dxc - trcumul[n].dxc + trcumul[n-2].dxc) ) - \
				cquad*freqnative*freqnative*(trsmoothed[n-1].dxc - trcumul[n-1].dxc)* \
				( 1 + nonlinfactor.dxc*fabsf(trsmoothed[n-1].dxc - trcumul[n-1].dxc) );

			// very light (2 frames interval) stabilization of zoom
			trsmoothed[n].dxx =  0.5f*(trcumul[n].dxx + trsmoothed[n-1].dxx);

			// dxy predictor:
			// double cutoff frequency for rotation
			trsmoothed[n].dxy = 2*trsmoothed[n-1].dxy - trsmoothed[n-2].dxy - \
				cdamp*2*freqnative*( trsmoothed[n-1].dxy - trsmoothed[n-2].dxy - trcumul[n-1].dxy + trcumul[n-2].dxy )* \
				( 1 + 0.5f*nonlinfactor.dxy/freqnative*fabsf(trsmoothed[n-1].dxy - trsmoothed[n-2].dxy - trcumul[n-1].dxy + trcumul[n-2].dxy) ) - \
				cquad*4*freqnative*freqnative*(trsmoothed[n-1].dxy - trcumul[n-1].dxy)* \
				( 1 + nonlinfactor.dxy*fabsf(trsmoothed[n-1].dxy - trcumul[n-1].dxy) );    // predictor
			// corrector, one iteration must be enough:
			trsmoothed[n].dxy = 2*trsmoothed[n-1].dxy - trsmoothed[n-2].dxy - \
				cdamp*2*freqnative*0.5f*( trsmoothed[n].dxy - trsmoothed[n-2].dxy - trcumul[n].dxy + trcumul[n-2].dxy )* \
				( 1 + 0.5f*nonlinfactor.dxy/freqnative*0.5f*fabsf(trsmoothed[n].dxy - trsmoothed[n-2].dxy - trcumul[n].dxy + trcumul[n-2].dxy) ) - \
				cquad*4*freqnative*freqnative*(trsmoothed[n-1].dxy - trcumul[n-1].dxy)* \
				( 1 + nonlinfactor.dxy*fabsf(trsmoothed[n-1].dxy - trcumul[n-1].dxy) );    // corrector, one iteration must be enough

			// dyx predictor:
			trsmoothed[n].dyx = -trsmoothed[n].dxy*(pixaspect/nfields)*(pixaspect/nfields); // must be consistent

			// dyc predictor:
			trsmoothed[n].dyc = 2*trsmoothed[n-1].dyc - trsmoothed[n-2].dyc - \
				cdamp*freqnative*( trsmoothed[n-1].dyc - trsmoothed[n-2].dyc - trcumul[n-1].dyc + trcumul[n-2].dyc )* \
				( 1 + 0.5f*nonlinfactor.dyc/freqnative*fabsf(trsmoothed[n-1].dyc - trsmoothed[n-2].dyc - trcumul[n-1].dyc + trcumul[n-2].dyc) ) - \
				cquad*freqnative*freqnative*(trsmoothed[n-1].dyc - trcumul[n-1].dyc)* \
				( 1 + nonlinfactor.dyc*fabsf(trsmoothed[n-1].dyc - trcumul[n-1].dyc) );    // predictor
			// corrector, one iteration must be enough:
			trsmoothed[n].dyc = 2*trsmoothed[n-1].dyc - trsmoothed[n-2].dyc - \
				cdamp*freqnative*0.5f*( trsmoothed[n].dyc - trsmoothed[n-2].dyc - trcumul[n].dyc + trcumul[n-2].dyc )* \
				( 1 + 0.5f*nonlinfactor.dyc/freqnative*0.5f*fabsf(trsmoothed[n].dyc - trsmoothed[n-2].dyc - trcumul[n].dyc + trcumul[n-2].dyc) ) - \
				cquad*freqnative*freqnative*(trsmoothed[n-1].dyc - trcumul[n-1].dyc)* \
				( 1 + nonlinfactor.dyc*fabsf(trsmoothed[n-1].dyc - trcumul[n-1].dyc) );    // corrector, one iteration must be enough


			// dyy
			trsmoothed[n].dyy = trsmoothed[n].dxx; //must be equal to dxx
		}


		if (addzoom) { // calculate and add adaptive zoom factor to fill borders (for all frames from base to ndest)

			azoom[nbase] = initzoom;
			azoom[nbase+1] = initzoom;
			azoomsmoothed[nbase] = initzoom;
			azoomsmoothed[nbase+1] = initzoom;
			for (n=nbase+2; n<=ndest; n++) {
				// get inverse transform
				inversetransform(trcumul[n], &trinv);
				// calculate difference between smoothed and original non-smoothed cumulative transform
				sumtransform(trinv, trsmoothed[n], &trcur);
				// find adaptive zoom factor
//				transform2motion (trcur, 1, xcenter, ycenter, pixaspect/nfields, &dxdif, &dydif, &rotdif, &zoomdif);
				azoom[n] = initzoom;
				float azoomtest = 1 + (trcur.dxc + trcur.dxy*ycenter)/xcenter; // xleft
				if (azoomtest < azoom[n]) azoom[n] = azoomtest;
				azoomtest = 1 - (trcur.dxc + trcur.dxx*vi.width + trcur.dxy*ycenter - vi.width)/xcenter; //xright
				if (azoomtest < azoom[n]) azoom[n] = azoomtest;
				azoomtest = 1 + (trcur.dyc + trcur.dyx*xcenter)/ycenter; // ytop
				if (azoomtest < azoom[n]) azoom[n] = azoomtest;
				azoomtest = 1 - (trcur.dyc + trcur.dyx*xcenter + trcur.dyy*vi.height - vi.height)/ycenter; //ybottom
				if (azoomtest < azoom[n]) azoom[n] = azoomtest;

				// limit zoom to max - added in v.1.4.0
//				if (fabsf(azoom[n]-1) > fabsf(zoommax)-1)
//					azoom[n] = 	2 - fabsf(zoommax) ;


				// smooth adaptive zoom
				// zoom time factor
				float zf = 1/(cutoff*tzoom);
				// predictor
				azoomsmoothed[n] = 2*azoomsmoothed[n-1] - azoomsmoothed[n-2] -
					zf*cdamp*freqnative*( azoomsmoothed[n-1] - azoomsmoothed[n-2] - azoom[n-1] + azoom[n-2] )
//					*( 1 + 0.5f*nonlinfactor.dxx/freqnative*fabsf(azoomsmoothed[n-1] - azoomsmoothed[n-2] - azoom[n-1] + azoom[n-2])) // disabled in v.1.4.0 for more smooth
					-zf*zf*cquad*freqnative*freqnative*(azoomsmoothed[n-1] - azoom[n-1])
//					*( 1 + nonlinfactor.dxx*fabsf(azoomsmoothed[n-1] - azoom[n-1]) )
				;
				// corrector, one iteration must be enough:
				azoomsmoothed[n] = 2*azoomsmoothed[n-1] - azoomsmoothed[n-2] -
					zf*cdamp*freqnative*0.5f*( azoomsmoothed[n] - azoomsmoothed[n-2] - azoom[n] + azoom[n-2] )
//					*( 1 + 0.5f*nonlinfactor.dxx/freqnative*0.5f*fabsf(azoomsmoothed[n] - azoomsmoothed[n-2] - azoom[n] + azoom[n-2]) )
					-zf*zf*cquad*freqnative*freqnative*(azoomsmoothed[n-1] - azoom[n-1])
//					*( 1 + nonlinfactor.dxx*fabsf(azoomsmoothed[n-1] - azoom[n-1]) )
					;
				zf = zf*0.7f; // slower zoom decreasing
				if (azoomsmoothed[n] > azoomsmoothed[n-1]) // added in v.1.4.0 for slower zoom decreasing
				{
				// predictor
				azoomsmoothed[n] = 2*azoomsmoothed[n-1] - azoomsmoothed[n-2] -
					zf*cdamp*freqnative*( azoomsmoothed[n-1] - azoomsmoothed[n-2] - azoom[n-1] + azoom[n-2] )
//					*( 1 + 0.5f*nonlinfactor.dxx/freqnative*fabsf(azoomsmoothed[n-1] - azoomsmoothed[n-2] - azoom[n-1] + azoom[n-2]))
					-zf*zf*cquad*freqnative*freqnative*(azoomsmoothed[n-1] - azoom[n-1])
//					*( 1 + nonlinfactor.dxx*fabsf(azoomsmoothed[n-1] - azoom[n-1]) )
				;
				// corrector, one iteration must be enough:
				azoomsmoothed[n] = 2*azoomsmoothed[n-1] - azoomsmoothed[n-2] -
					zf*cdamp*freqnative*0.5f*( azoomsmoothed[n] - azoomsmoothed[n-2] - azoom[n] + azoom[n-2] )
//					*( 1 + 0.5f*nonlinfactor.dxx/freqnative*0.5f*fabsf(azoomsmoothed[n] - azoomsmoothed[n-2] - azoom[n] + azoom[n-2]) )
					-zf*zf*cquad*freqnative*freqnative*(azoomsmoothed[n-1] - azoom[n-1])
//					*( 1 + nonlinfactor.dxx*fabsf(azoomsmoothed[n-1] - azoom[n-1]) )
					;
				}
				//			azoomsmoothed[n] = azoomcumul[n]; // debug - no azoom smoothing
				if(azoomsmoothed[n] > 1)
					azoomsmoothed[n] = 1;  // not decrease image size
				// make zoom transform
				motion2transform (0, 0, 0, azoomsmoothed[n], pixaspect/nfields, xcenter, ycenter, 1, 1.0, &trtemp); // added in v.1.5.0
				// get non-adaptive image zoom from transform
//				transform2motion (trsmoothed[n], 1, xcenter, ycenter, pixaspect/nfields, &dxdif, &dydif, &rotdif, &zoomdif); // disabled in v.1.5.0
				// modify transform with adaptive zoom added
//				motion2transform (dxdif, dydif, rotdif, zoomdif*azoomsmoothed[n], pixaspect/nfields,  xcenter,  ycenter, 1, 1.0, &trsmoothed[n]); // replaced in v.1.5.0 by:
				sumtransform (trsmoothed[n],trtemp,  &trsmoothed[n]); // added v.1.5.0

			}
		}
		else
		{
			motion2transform (0, 0, 0, initzoom, pixaspect/nfields, xcenter, ycenter, 1, 1.0, &trtemp); // added in v.1.7
			sumtransform (trsmoothed[ndest],trtemp,  &trsmoothed[ndest]); // added v.1.7
		}

		// calculate difference between smoothed and original non-smoothed cumulative tranform
		// it will be used as stabilization values

		inversetransform(trcumul[ndest], &trinv);
		sumtransform( trinv,trsmoothed[ndest], ptrdif);
}



void DePanStabilize::Average(int nbase, int ndest, int nmax, transform * ptrdif)
{
	int n;
	transform trinv, trcur, trtemp;

			float norm = 0;
			trsmoothed[ndest].dxc = 0;
			trsmoothed[ndest].dyc = 0;
			trsmoothed[ndest].dxy = 0;
			for (n=nbase; n<ndest; n++)
			{
				trsmoothed[ndest].dxc += trcumul[n].dxc*wint[ndest-n];
				trsmoothed[ndest].dyc += trcumul[n].dyc*wint[ndest-n];
				trsmoothed[ndest].dxy += trcumul[n].dxy*wint[ndest-n];
				norm += wint[ndest-n];
			}
			for (n=ndest; n<=nmax; n++)
			{
				trsmoothed[ndest].dxc += trcumul[n].dxc*wint[n-ndest];
				trsmoothed[ndest].dyc += trcumul[n].dyc*wint[n-ndest];
				trsmoothed[ndest].dxy += trcumul[n].dxy*wint[n-ndest];
				norm += wint[n-ndest];
			}
			trsmoothed[ndest].dxc /= norm;
			trsmoothed[ndest].dyc /= norm;
			trsmoothed[ndest].dxy /= norm;
			trsmoothed[ndest].dyx = -trsmoothed[ndest].dxy*(pixaspect/nfields)*(pixaspect/nfields); // must be consistent
			norm = 0;
			trsmoothed[ndest].dxx = 0;
			for (n=std::max(nbase, ndest-1); n<ndest; n++) { // very short interval
				trsmoothed[ndest].dxx += trcumul[n].dxx*wint[ndest-n];
				norm += wint[ndest-n];
			}
			for (n=ndest; n<=std::min(nmax,ndest+1); n++) {
				trsmoothed[ndest].dxx += trcumul[n].dxx*wint[n-ndest];
				norm += wint[n-ndest];
			}
			trsmoothed[ndest].dxx /= norm;
			trsmoothed[ndest].dyy = trsmoothed[ndest].dxx;

//			motion2transform (0, 0, 0, initzoom, pixaspect/nfields, xcenter, ycenter, 1, 1.0, &trtemp); // added in v.1.7
//			sumtransform (trsmoothed[ndest],trtemp,  &trsmoothed[ndest]); // added v.1.7

			if (addzoom) { // calculate and add adaptive zoom factor to fill borders (for all frames from base to ndest)

				int nbasez = std::max(nbase, ndest-winfzsize);
				int nmaxz = std::min(nmax, ndest+winrzsize);
            	// symmetrical
 //               nmaxz = ndest + std::min(nmaxz-ndest, ndest-nbasez);
 //               nbasez = ndest - std::min(nmaxz-ndest, ndest-nbasez);

				azoom[nbasez] = initzoom;
				for (n=nbasez+1; n<=nmaxz; n++) {
					// get inverse transform
					inversetransform(trcumul[n], &trinv);
					// calculate difference between smoothed and original non-smoothed cumulative transform
//					sumtransform(trinv, trsmoothed[n], &trcur);
					sumtransform(trinv, trcumul[n], &trcur);
					// find adaptive zoom factor
					azoom[n] = initzoom;
					float azoomtest = 1 + (trcur.dxc + trcur.dxy*ycenter)/xcenter; // xleft
					if (azoomtest < azoom[n]) azoom[n] = azoomtest;
					azoomtest = 1 - (trcur.dxc + trcur.dxx*vi.width + trcur.dxy*ycenter - vi.width)/xcenter; //xright
					if (azoomtest < azoom[n]) azoom[n] = azoomtest;
					azoomtest = 1 + (trcur.dyc + trcur.dyx*xcenter)/ycenter; // ytop
					if (azoomtest < azoom[n]) azoom[n] = azoomtest;
					azoomtest = 1 - (trcur.dyc + trcur.dyx*xcenter + trcur.dyy*vi.height - vi.height)/ycenter; //ybottom
					if (azoomtest < azoom[n]) azoom[n] = azoomtest;
//					azoom[n] = initzoom;
				}

					// smooth adaptive zoom
					// zoom time factor
//					zf = 1/(cutoff*tzoom);

				norm = 0;
				azoomsmoothed[ndest] = 0.0;
				for (n=nbasez; n<ndest; n++)
				{
					azoomsmoothed[ndest] += azoom[n]*winfz[ndest-n]; // fall
					norm += winfz[ndest-n];
				}
				for (n=ndest; n<=nmaxz; n++)
				{
					azoomsmoothed[ndest] += azoom[n]*winrz[n-ndest]; // rize
					norm += winrz[n-ndest];
				}
				azoomsmoothed[ndest] /= norm;
//		sprintf(debugbuf,"DePanStabilize: nbase=%d ndest=%d nmax=%d z=%f zs=%f norm=%f\n", nbase, ndest, nmax, azoom[ndest], azoomsmoothed[ndest], norm );
//		OutputDebugString(debugbuf);

//					zf = zf*0.7f; // slower zoom decreasing
//					if (azoomsmoothed[n] > azoomsmoothed[n-1]) // added in v.1.4.0 for slower zoom decreasing
//					{
//					}

					//azoomsmoothed[ndest] = azoom[ndest]; // debug - no azoom smoothing

					if(azoomsmoothed[ndest] > 1)
						azoomsmoothed[ndest] = 1;  // not decrease image size
					// make zoom transform
					motion2transform (0, 0, 0, azoomsmoothed[ndest], pixaspect/nfields, xcenter, ycenter, 1, 1.0, &trtemp);
					sumtransform (trsmoothed[ndest],trtemp,  &trsmoothed[ndest]);

	//			}
			}
			else // no addzoom
			{
				motion2transform (0, 0, 0, initzoom, pixaspect/nfields, xcenter, ycenter, 1, 1.0, &trtemp); // added in v.1.7
				sumtransform (trsmoothed[ndest],trtemp,  &trsmoothed[ndest]); // added v.1.7
			}
		// calculate difference between smoothed and original non-smoothed cumulative tranform
		// it will be used as stabilization values

		inversetransform(trcumul[ndest], &trinv);
		sumtransform( trinv,trsmoothed[ndest], ptrdif);
}



void DePanStabilize::InertialLimit(float *dxdif, float *dydif, float *zoomdif, float *rotdif, int ndest, int *nbase)
{
		// limit max motion corrections
		if ( !(isfinite(*dxdif)) ) // check added in v.1.1.3
		{// infinite or NAN
				*dxdif = 0;
				*dydif = 0;
				*zoomdif = initzoom;
				*rotdif = 0;
				*nbase = ndest;
		}
		else if (fabsf(*dxdif) > fabsf(dxmax))
		{
			if (dxmax>=0 )
			{
				*dxdif = *dxdif >= 0 ? sqrt(*dxdif*dxmax) : -sqrt(-*dxdif*dxmax); // soft limit v.1.8.2
			}
			else
			{
					*dxdif = 0;
					*dydif = 0;
					*zoomdif = initzoom;
					*rotdif = 0;
					*nbase = ndest;
			}
		}

		if ( !(isfinite(*dydif)) )
		{// infinite or NAN
				*dxdif = 0;
				*dydif = 0;
				*zoomdif = initzoom;
				*rotdif = 0;
				*nbase = ndest;
		}
		else if (fabsf(*dydif) > fabsf(dymax))
		{
			if (dymax>=0 )
			{
				*dydif = *dydif >= 0 ? sqrt(*dydif*dymax) : -sqrt(-*dydif*dymax); // soft limit v.1.8.2
			}
			else
			{
				*dxdif = 0;
				*dydif = 0;
				*zoomdif = initzoom;
				*rotdif = 0;
				*nbase = ndest;
			}
		}

		if ( !(isfinite(*zoomdif)) )
		{// infinite or NAN
				*dxdif = 0;
				*dydif = 0;
				*zoomdif = initzoom;
				*rotdif = 0;
				*nbase = ndest;
		}
		else if (fabsf(*zoomdif-1) > fabsf(zoommax)-1)
		{
			if (zoommax>=0 )
			{
				*zoomdif = *zoomdif >= 1 ? 1 + sqrt(fabsf(*zoomdif-1)*fabsf(zoommax-1)) : 1 - sqrt(fabsf(*zoomdif-1)*fabsf(zoommax-1)); // soft limit v.1.8.2
			}
			else
			{
				*dxdif = 0;
				*dydif = 0;
				*zoomdif = initzoom;
				*rotdif = 0;
				*nbase = ndest;
			}
		}

		if ( !(isfinite(*rotdif)) )
		{// infinite or NAN
				*dxdif = 0;
				*dydif = 0;
				*zoomdif = initzoom;
				*rotdif = 0;
				*nbase = ndest;
		}
		else if (fabsf(*rotdif) > fabsf(rotmax))
		{
			if (rotmax>=0 )
			{
				*rotdif = *rotdif >= 0 ? sqrt(*rotdif*rotmax) : -sqrt(-*rotdif*rotmax); // soft limit v.1.8.2
			}
			else
			{
				*dxdif = 0;
				*dydif = 0;
				*zoomdif = initzoom;
				*rotdif = 0;
				*nbase = ndest;
			}
		}
}

float DePanStabilize::Averagefraction(float dxdif, float dydif, float zoomdif, float rotdif)
{
	float fractionx = fabsf(dxdif) / fabsf(dxmax);
	float fractiony = fabsf(dydif) / fabsf(dymax);
	float fractionz = fabsf(zoomdif-1) / fabsf((fabsf(zoommax)-1));
	float fractionr = fabsf(rotdif) / fabsf(rotmax);

	float fraction = std::max(fractionx, fractiony);
	fraction = std::max(fraction, fractionz);
	fraction = std::max(fraction, fractionr);
	return fraction;

}

// ****************************************************************************
//
PVideoFrame __stdcall DePanStabilize::GetFrame(int ndest, IScriptEnvironment* env) {
// This is the implementation of the GetFrame function.
// See the header definition for further info.
	PVideoFrame src, dst, dataframe;
	BYTE *dstp, *dstpU, *dstpV;
	const BYTE *srcp, *srcpU, *srcpV;
	int dst_pitch, dst_pitchUV;
	int src_width, src_height, src_pitch, src_pitchUV, src_widthUV,src_heightUV;
	const BYTE *datap;
	float dxdif, dydif,zoomdif,rotdif;
	int border, borderUV;
	int n, error;
	char messagebuf[32];
//	char debugbuf[100];
//	int nfields;
	int nbasenew;
	int nprev, nnext;
	int notfilled;
//	float azoomtest;

//	float zf;

	transform trcur, trnull, trinv, trdif, trtemp, trY, trUV;

	float zoom = 1; // make null transform
	motion2transform (0, 0, 0, zoom, 1, 0, 0, 1, 1.0, &trnull);

	int isYUY2 = vi.IsYUY2();//v1.6
	int xmsg;

	// correction for fieldbased
//	if (fieldbased != 0) 	nfields = 2;
//	else nfields = 1;

	// This code deals with YV12 colourspace where the Y, U and V information are
	// stored in completely separate memory areas

// ---------------------------------------------------------------------------
	// Get motion info from the DePanData clip frame

	if (method == 0)
		nbasenew = int(ndest - 10*fps/cutoff); // range of almost full stablization - increased in v 1.4.0
	else
		nbasenew = ndest - radius;

	if (nbasenew < 0) nbasenew = 0;
	if (nbasenew > nbase || method != 0) nbase = nbasenew; // increase base to limit range
	if (nbase > ndest) nbase = nbasenew; // correction after backward scan

	int nmax;
	if (method == 0) // inertial
		nmax = ndest;
	else
		nmax = std::min(ndest + radius, vi.num_frames-1); // max n to take into account

	// get motion info about frames in interval from begin source to dest in reverse order
	for (n = nbase; n <= ndest; n++) {

		if (motionx[n] == MOTIONUNKNOWN) { // motion data is unknown for needed frame
			// note: if inputlogfile has been read, all motion data is always known

			// Request frame n from the DePanData clip.
			// It must contains coded motion data in framebuffer
			dataframe = DePanData->GetFrame(n, env);
			// get pointer to data frame
			datap = dataframe->GetReadPtr();
			// get motiondata from DePanData clip framebuffer
			error = read_depan_data(datap, motionx, motiony, motionzoom, motionrot, n);
			// check if correct
			if (error != 0) env->ThrowError("DePanStabilize: data clip is NOT good DePanEstimate clip !");
		}

//		if (motionx[n] == MOTIONBAD ) break; // if strictly =0,  than no good
	}

	for (n = ndest; n >= nbase; n--) {

		if (motionx[n] == MOTIONBAD ) break; // if strictly =0,  than no good
	}

	// limit frame search range
	if (n > nbase) {
		nbase = n;  // set base frame to new scene start if found
	}

//		sprintf(debugbuf,"DePanStabilize: nbase=%d ndest=%d nmax=%d\n", nbase, ndest, nmax);
//		OutputDebugString(debugbuf);

	for (n = ndest+1; n <= nmax; n++) {

		if (motionx[n] == MOTIONUNKNOWN) { // motion data is unknown for needed frame
			// note: if inputlogfile has been read, all motion data is always known

			// Request frame n from the DePanData clip.
			// It must contains coded motion data in framebuffer
			dataframe = DePanData->GetFrame(n, env);
			// get pointer to data frame
			datap = dataframe->GetReadPtr();
			// get motiondata from DePanData clip framebuffer
			error = read_depan_data(datap, motionx, motiony, motionzoom, motionrot, n);
			// check if correct
			if (error != 0) env->ThrowError("DePanStabilize: data clip is NOT good DePanEstimate clip !");
		}

		if (motionx[n] == MOTIONBAD ) break; // if strictly =0,  than no good
	}

//		sprintf(debugbuf,"DePanStabilize: nbase=%d ndest=%d nmax=%d n=%d\n", nbase, ndest, nmax, n);
//		OutputDebugString(debugbuf);
	// limit frame search range
	if (n < nmax) {
		nmax = std::max(n-1, ndest);  // set max frame to new scene start-1 if found
	}

	if (method != 0)
	{	// symmetrical
		nmax = ndest + std::min(nmax-ndest, ndest-nbase);
		nbase = ndest - std::min(nmax-ndest, ndest-nbase);
	}

//		sprintf(debugbuf,"DePanStabilize: nbase=%d ndest=%d nmax=%d\n", nbase, ndest, nmax);
//		OutputDebugString(debugbuf);

	motion2transform (0, 0, 0, initzoom, pixaspect/nfields, xcenter, ycenter, 1, 1.0, &trtemp); // added in v.1.7
	if (nbase == ndest && method==0) { // we are at new scene start,
//		trdif null transform
		sumtransform (trtemp, trnull, &trdif); //v1.7
	}
	else if (nbase == ndest-1 && method==0) { // first next frame after base
//		trdif null transform
		sumtransform (trtemp, trnull, &trdif); //v1.7
	}
	else { // prepare stabilization data by estimation and smoothing of cumulative motion

	  // cumulative transform (position) for all sequence from base

	  // base as null
	  sumtransform(trnull, trnull, &trcumul[nbase]);// v.1.8.1

	  // get cumulative transforms from base to ndest
	  for (n=nbase+1; n<=nmax; n++) {
			motion2transform (motionx[n], motiony[n], motionrot[n], motionzoom[n], pixaspect/nfields,  xcenter,  ycenter, 1, 1.0, &trcur);
			sumtransform (trcumul[n-1], trcur, &trcumul[n]);
	  }

	  if (method == 0)// (inertial)
	  {
		DePanStabilize::Inertial(nbase, ndest, &trdif);
		// summary motion from summary transform
		transform2motion (trdif, 1, xcenter, ycenter, pixaspect/nfields, &dxdif, &dydif, &rotdif, &zoomdif);
		// fit last - decrease motion correction near end of clip - added in v.1.2.0

		if (vi.num_frames < fitlast + ndest +1 && method == 0)
		{
			float endFactor = ((float)(vi.num_frames - ndest - 1))/fitlast; // decrease factor
			dxdif *=  endFactor;
			dydif *=  endFactor;
			rotdif *=  endFactor;
			zoomdif =  initzoom + (zoomdif-initzoom)*endFactor;
		}

		DePanStabilize::InertialLimit(&dxdif, &dydif, &zoomdif, &rotdif, ndest, &nbase);
	  }
	  else // method == 1 (windowed average)
	  {
		  int radius1 = radius;
		  for (int iter=0; iter<1; iter++)
		  {
			DePanStabilize::Average(nbase, ndest, nmax, &trdif);
			// summary motion from summary transform
			transform2motion (trdif, 1, xcenter, ycenter, pixaspect/nfields, &dxdif, &dydif, &rotdif, &zoomdif);
			float fraction = DePanStabilize::Averagefraction(dxdif, dydif, zoomdif, rotdif);
	/*		if (fraction>0.5)
			{
				// decrease radius
				radius1 = radius1*0.9;
				nbase = std::max(nbase, ndest - radius1);
				nmax = std::min(nmax, ndest + radius1);
				// update wint and may be winz
					float PI = 3.14159265258;
					for (int i=0; i<radius1; i++)
						wint[i] = cosf(i*0.5*PI/radius1);
					for (int i=radius1; i<=wintsize; i++)
						wint[i] = 0;
			}
			else
			{
				if (radius1 != radius) // was decreased
					break;
				// increase radius
				radius1 = std::min(radius + 1, wintsize);
				if (radius1 != radius)
				{
					// update wint and may be winz
					float PI = 3.14159265258;
					for (int i=0; i<radius1; i++)
						wint[i] = cosf(i*0.5*PI/radius1);
					for (int i=radius1; i<=wintsize; i++)
						wint[i] = 0;
				}
				break; // exit for loop
			}
*/			// new iteration cycle
		  }
		radius = radius1;
	  }


//		char debugbuf[96]; // buffer for debugview utility
//		int debug=1;
//		if (debug != 0) { // debug mode
			// output data for debugview utility
//			sprintf(debugbuf,"DePanStabilize: frame %d dx=%7.2f dy=%7.2f zoom=%7.5f\n", ndest, dxdif, dydif, zoomdif);
//			OutputDebugString(debugbuf);
//		}


		// summary motion from summary transform after max correction
		motion2transform (dxdif, dydif, rotdif, zoomdif, pixaspect/nfields,  xcenter,  ycenter, 1, 1.0, &trdif);
	}

	// final motion dif for info
	transform2motion (trdif, 1, xcenter, ycenter, pixaspect/nfields, &dxdif, &dydif, &rotdif, &zoomdif);




// ---------------------------------------------------------------------------
	// Construct a new frame based on the information of the current frame
	// contained in the "vi" struct.
	dst = env->NewVideoFrame(vi);
	// Request a Write pointer from the newly created destination image.
	dstp = dst->GetWritePtr();
	dst_pitch = dst->GetPitch();
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
	//output motion to AviSynth (float) variables - added in v.1.8 as requested by AI
	if (vdx != "")
	{
		vdx_val = dxdif; // store to static place
		env->SetVar(vdx, vdx_val); // by reference!
	}
	if (vdy != "")
	{
		vdy_val = dydif;
		env->SetVar(vdy, vdy_val);
	}
	if (vzoom != "")
	{
		vzoom_val = zoomdif;
	}
		env->SetVar(vzoom, vzoom_val);
	if (vrot != "")
	{
		vrot_val = rotdif;
		env->SetVar(vrot, vrot_val);
	}

//--------------------------------------------------------------------------
	// Ready to make motion stabilization,
	if (isYUY2)
	{
		;//nothing
	}
	else // YV12
	{
		// additional info on the U and V planes
		dst_pitchUV = dst->GetPitch(PLANAR_U);	// The pitch,height and width information
		dstpU = dst->GetWritePtr(PLANAR_U);
		dstpV = dst->GetWritePtr(PLANAR_V);
	}




// --------------------------------------------------------------------
	// use some previous frame to fill borders
	notfilled = 1;  // init as not filled (borders by neighbor frames)
	float dxt1, dyt1, rott1, zoomt1;
	float dabsmin;
//	char debugbuf[80];
	if (fillprev > 0) {
		nprev = ndest-fillprev; // get prev frame number
		if ( nprev < nbase ) nprev = nbase; //  prev distance not exceed base
		int nprevbest = nprev;
		dabsmin = 10000;
		trY = trdif; // luma transform

		for (n=ndest-1; n>=nprev; n--) {  // summary inverse transform
			motion2transform (motionx[n+1], motiony[n+1], motionrot[n+1], motionzoom[n+1], pixaspect/nfields,  xcenter,  ycenter, 1, 1.0, &trcur);
			trtemp = trY;
			nprevbest = n;
			sumtransform ( trtemp,trcur, &trY);
			transform2motion (trY, 1, xcenter, ycenter, pixaspect/nfields, &dxt1, &dyt1, &rott1, &zoomt1);
			if ((fabs(dxt1)+fabs(dyt1) + ndest-n)<dabsmin) { // most centered and nearest
				dabsmin = fabs(dxt1)+fabs(dyt1)+ ndest-n;
				nprevbest = n;
			}
		}

		// get original previous source frame
		src = child->GetFrame(nprevbest, env);

		// Request a Read pointer from the source frame.
		// This will return the position of the upperleft pixel in YV12 images,
		// frame info
		srcp = src->GetReadPtr();
		src_width = src->GetRowSize();
		src_height = src->GetHeight();
		// Requests pitch (length of a line) of the destination image.
		// (short version - pitch is always equal to or greater than width to allow for seriously fast assembly code)
		src_pitch = src->GetPitch();

		if (isYUY2)
		{
		// create planes from YUY2
			YUY2ToPlanes(srcp, src_height, src_width, src_pitch, srcplaneY, planeYpitch, srcplaneU, planeUVpitch, srcplaneV, planeUVpitch);
			// Process Luma Plane Y
			border = 0;  // luma=0, black
			compensate_plane_nearest (dstplaneY,  planeYpitch, srcplaneY,  planeYpitch,  planeYwidth, src_height, trY, mirror, border, work2width4356, blur);
			// result transform for U, V planes (half)
			trUV.dxc = trY.dxc/2;
			trUV.dxx = trY.dxx;
			trUV.dxy = trY.dxy/2;
			trUV.dyc = trY.dyc;
			trUV.dyx = trY.dyx*2;
			trUV.dyy = trY.dyy;

			borderUV = 128; // border color = grey if both U,V=128
			compensate_plane_nearest (dstplaneU,  planeUVpitch, srcplaneU,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
			compensate_plane_nearest (dstplaneV,  planeUVpitch, srcplaneV,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
		}
		else // YV12
		{
			// Process Luma Plane Y
			border = 0;  // luma=0, black
			// move src frame plane by vector to partially motion compensated position
			compensate_plane_nearest (dstp,  dst_pitch, srcp,  src_pitch,  src_width, src_height, trY, mirror, border, work2width4356, blur);
			// This section of code deals with the U and V planes of planar formats (e.g. YV12)
			src_pitchUV = src->GetPitch(PLANAR_U);	// is guaranted to be the same for both
			src_widthUV = src->GetRowSize(PLANAR_U);	// the U and V planes so we only the U
			src_heightUV = src->GetHeight(PLANAR_U);	//plane values and use them for V as well
			srcpU = src->GetReadPtr(PLANAR_U);
			srcpV = src->GetReadPtr(PLANAR_V);

			// result transform for U, V planes (half)
			trUV.dxc = trY.dxc/2;
			trUV.dxx = trY.dxx;
			trUV.dxy = trY.dxy;
			trUV.dyc = trY.dyc/2;
			trUV.dyx = trY.dyx;
			trUV.dyy = trY.dyy;

			borderUV = 128; // border color = grey if both U,V=128

			//	compensate U plane
			// use only fast nearest  method
			compensate_plane_nearest (dstpU,  dst_pitchUV, srcpU,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane

			//	compensate V plane
			compensate_plane_nearest (dstpV,  dst_pitchUV, srcpV,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror,borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
		}
		notfilled = 0;  // mark as FILLED

	}
// --------------------------------------------------------------------
	// use next frame to fill borders
	if (fillnext > 0 ) {
		nnext = ndest + fillnext;
		if (nnext >= vi.num_frames) nnext = vi.num_frames-1;
		int nnextbest = nnext;
		dabsmin = 1000;
		trY = trdif; // luma transform for current frame

		// get motion info about frames in interval from begin source to dest in reverse order
		for (n = ndest+1; n <= nnext; n++) {

			if (motionx[n] == MOTIONUNKNOWN) { // motion data is unknown for needed frame
				// note: if inputlogfile has been read, all motion data is always known

				// Request frame n from the DePanData clip.
				// It must contains coded motion data in framebuffer
				dataframe = DePanData->GetFrame(n, env);
				// get pointer to data frame
				datap = dataframe->GetReadPtr();
				// get motiondata from DePanData clip framebuffer
				error = read_depan_data(datap, motionx, motiony, motionzoom, motionrot, n);
				// check if correct
				if (error != 0) env->ThrowError("DePan: data clip is NOT good DePanEstimate clip !");
			}

			if (motionx[n] != MOTIONBAD ) { //if good
				motion2transform (motionx[n], motiony[n], motionrot[n], motionzoom[n], pixaspect/nfields,  xcenter,  ycenter, 1, 1.0, &trcur);
				inversetransform(trcur, &trinv);
				trtemp = trY;
				sumtransform ( trinv,trtemp, &trY);
				transform2motion (trY, 1, xcenter, ycenter, pixaspect/nfields, &dxt1, &dyt1, &rott1, &zoomt1);
				if ((fabs(dxt1)+fabs(dyt1) + n-ndest) <dabsmin) { // most centered and nearest
					dabsmin = fabs(dxt1)+fabs(dyt1)+ n-ndest;
					nnextbest = n;
				}
			}
			else { // bad
				nnextbest = n-1;  // limit fill frame to last good
				break;
			}
		}

//		motion2transform (motionx[nnext], motiony[nnext], motionrot[nnext], motionzoom[nnext], pixaspect/nfields,  xcenter,  ycenter, 1, 1.0, &trcur);

		// get original previous source frame
		src = child->GetFrame(nnextbest, env);

		// Request a Read pointer from the source frame.
		// This will return the position of the upperleft pixel in YV12 images,
		// frame info
		srcp = src->GetReadPtr();
		src_width = src->GetRowSize();
		src_height = src->GetHeight();
		src_pitch = src->GetPitch();

		if(isYUY2)
		{
			// create planes from YUY2
			YUY2ToPlanes(srcp, src_height, src_width, src_pitch, srcplaneY, planeYpitch, srcplaneU, planeUVpitch, srcplaneV, planeUVpitch);
			// Process Luma Plane Y
			if (notfilled == 0) {
				border = -1;  // negative - borders is filled by prev, will not fill by black!
			}
			else {
				border = 0; // no prev, then will fill by black
			}

			// move src frame plane by vector to partially motion compensated position
			compensate_plane_nearest (dstplaneY,  planeYpitch, srcplaneY,  planeYpitch,  planeYwidth, src_height, trY, mirror, border, work2width4356, blur);
			// result transform for U, V planes (half)
			trUV.dxc = trY.dxc/2;
			trUV.dxx = trY.dxx;
			trUV.dxy = trY.dxy/2;
			trUV.dyc = trY.dyc;
			trUV.dyx = trY.dyx*2;
			trUV.dyy = trY.dyy;

			if (notfilled == 0) {
				borderUV = -1;  // negative - borders is filled by prev, not by black!
			}
			else {
				borderUV = 128; // no prev, fill by black
			}
			compensate_plane_nearest (dstplaneU,  planeUVpitch, srcplaneU,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
			compensate_plane_nearest (dstplaneV,  planeUVpitch, srcplaneV,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
		}
		else //YV12
		{
			src_pitchUV = src->GetPitch(PLANAR_U);	// is guaranted to be the same for both
			src_widthUV = src->GetRowSize(PLANAR_U);	// the U and V planes so we only the U
			src_heightUV = src->GetHeight(PLANAR_U);	//plane values and use them for V as well
			srcpU = src->GetReadPtr(PLANAR_U);
			srcpV = src->GetReadPtr(PLANAR_V);

			// Process Luma Plane Y
			if (notfilled == 0) {
				border = -1;  // negative - borders is filled by prev, will not fill by black!
			}
			else {
				border = 0; // no prev, then will fill by black
			}

			// move src frame plane by vector to partially motion compensated position
				compensate_plane_nearest (dstp,  dst_pitch, srcp,  src_pitch,  src_width, src_height, trY, mirror*notfilled, border, work2width4356, blur);

			// This section of code deals with the U and V planes of planar formats (e.g. YV12)

			// result transform for U, V planes (half)
			trUV.dxc = trY.dxc/2;
			trUV.dxx = trY.dxx;
			trUV.dxy = trY.dxy;
			trUV.dyc = trY.dyc/2;
			trUV.dyx = trY.dyx;
			trUV.dyy = trY.dyy;


			if (notfilled == 0) {
				borderUV = -1;  // negative - borders is filled by prev, not by black!
			}
			else {
				borderUV = 128; // no prev, fill by black
			}

			//	compensate U plane
			compensate_plane_nearest (dstpU,  dst_pitchUV, srcpU,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror*notfilled, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
			//	compensate V plane
			compensate_plane_nearest (dstpV,  dst_pitchUV, srcpV,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror*notfilled, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
		}

		notfilled = 0; // mark as filled

	}
// --------------------------------------------------------------------
	// get original current source frame
		trY = trdif; // restore transform

		src = child->GetFrame(ndest, env);

		// Request a Read pointer from the source frame.
		// This will return the position of the upperleft pixel in YV12 images,
		// frame info
		srcp = src->GetReadPtr();
		src_width = src->GetRowSize();
		src_height = src->GetHeight();
		// Requests pitch (length of a line) of the destination image.
		// (short version - pitch is always equal to or greater than width to allow for seriously fast assembly code)
		src_pitch = src->GetPitch();

		if (isYUY2)
		{
			// create planes from YUY2
			YUY2ToPlanes(srcp, src_height, src_width, src_pitch, srcplaneY, planeYpitch, srcplaneU, planeUVpitch, srcplaneV, planeUVpitch);
			// Process Luma Plane Y

			if (notfilled == 0) {
				border = -1;  // negative - borders is filled by prev, not by black!
			}
			else border = 0; // no prev, fill by black
			// move src frame plane by vector to motion compensated position
			if (subpixel == 2) { // bicubic interpolation
				compensate_plane_bicubic (dstplaneY,  planeYpitch, srcplaneY,  planeYpitch,  planeYwidth, src_height, trY, mirror, border, work2width4356, blur);
			}
			else if (subpixel == 1) { // bilinear interpolation
				compensate_plane_bilinear (dstplaneY,  planeYpitch, srcplaneY,  planeYpitch,  planeYwidth, src_height, trY, mirror, border, work2width4356, blur);
			}
			else {  //subpixel=0, nearest pixel accuracy
				compensate_plane_nearest (dstplaneY,  planeYpitch, srcplaneY,  planeYpitch,  planeYwidth, src_height, trY, mirror, border, work2width4356, blur);
			}
			// result transform for U, V planes (half)
			trUV.dxc = trY.dxc/2;
			trUV.dxx = trY.dxx;
			trUV.dxy = trY.dxy/2;
			trUV.dyc = trY.dyc;
			trUV.dyx = trY.dyx*2;
			trUV.dyy = trY.dyy;

			if (notfilled == 0) {
				borderUV = -1;  // negative - borders is filled by prev, not by black!
			}
			else {
				borderUV = 128; // no prev, fill by black
			}
			// Process U plane
			if (subpixel == 2) { // bicubic interpolation
				compensate_plane_bicubic (dstplaneU,  planeUVpitch, srcplaneU,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2);
			}
			else if (subpixel == 1) {  // bilinear interpolation
				compensate_plane_bilinear (dstplaneU,  planeUVpitch, srcplaneU,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
			}
			else {  //subpixel=0, nearest pixel accuracy
				compensate_plane_nearest (dstplaneU,  planeUVpitch, srcplaneU,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
			}

			// Process V plane
			if (subpixel == 2) { // bicubic interpolation
				compensate_plane_bicubic (dstplaneV,  planeUVpitch, srcplaneV,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2);
			}
			else if (subpixel == 1) {  // bilinear interpolation
				compensate_plane_bilinear (dstplaneV,  planeUVpitch, srcplaneV,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
			}
			else {  //subpixel=0, nearest pixel accuracy
				compensate_plane_nearest (dstplaneV,  planeUVpitch, srcplaneV,  planeUVpitch,  planeUVwidth, src_height, trUV, mirror, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
			}

			// create YUY2 from planes
			PlanesToYUY2(dstp, src_height, src_width, dst_pitch, dstplaneY, planeYpitch, dstplaneU, planeUVpitch, dstplaneV, planeUVpitch);

			xmsg = (nfields != 1) ? (ndest%2)*15 : 0; // x-position of odd fields message
			if (info) { // show text info on frame image
				sprintf(messagebuf," DePanStabilize");
				DrawString(dst,xmsg,1,messagebuf, isYUY2);
				sprintf(messagebuf," frame=%7d", ndest);
				DrawString(dst,xmsg,2,messagebuf, isYUY2);
				if (nbase == ndest) sprintf(messagebuf," BASE!=%7d", nbase); // CAPITAL letters
				else sprintf(messagebuf," base =%7d", nbase);
				DrawString(dst,xmsg,3,messagebuf, isYUY2);
				sprintf(messagebuf," dx  =%7.2f", dxdif);
				DrawString(dst,xmsg,4,messagebuf, isYUY2);
				sprintf(messagebuf," dy  =%7.2f", dydif);
				DrawString(dst,xmsg,5,messagebuf, isYUY2);
				sprintf(messagebuf," zoom=%7.5f", zoomdif);
				DrawString(dst,xmsg,6,messagebuf, isYUY2);
				sprintf(messagebuf," rot= %7.3f", rotdif);
				DrawString(dst,xmsg,7,messagebuf, isYUY2);
			}


		}
		else // YV12
		{
			src_pitchUV = src->GetPitch(PLANAR_U);	// is guaranted to be the same for both
			src_widthUV = src->GetRowSize(PLANAR_U);	// the U and V planes so we only the U
			src_heightUV = src->GetHeight(PLANAR_U);	//plane values and use them for V as well
			srcpU = src->GetReadPtr(PLANAR_U);
			srcpV = src->GetReadPtr(PLANAR_V);


			// Process Luma Plane Y

			if (notfilled == 0) {
				border = -1;  // negative - borders is filled by prev, not by black!
			}
			else border = 0; // no prev, fill by black

			// move src frame plane by vector to partially motion compensated position
			if (subpixel == 2) { // bicubic interpolation
				compensate_plane_bicubic (dstp,  dst_pitch, srcp,  src_pitch,  src_width, src_height, trY, mirror*notfilled, border, work2width4356, blur );
			}
			else if (subpixel == 1) {  // bilinear interpolation
				compensate_plane_bilinear (dstp,  dst_pitch, srcp,  src_pitch,  src_width, src_height, trY, mirror*notfilled, border, work2width4356, blur);//
			}
			else {  //subpixel=0, nearest pixel accuracy
				compensate_plane_nearest (dstp,  dst_pitch, srcp,  src_pitch,  src_width, src_height, trY, mirror*notfilled, border, work2width4356, blur);
			}

			xmsg = (nfields != 1) ? (ndest%2)*15 : 0; // x-position of odd fields message
			if (info) { // show text info on frame image
				sprintf(messagebuf," DePanStabilize");
				DrawString(dst,xmsg,1,messagebuf, isYUY2);
				sprintf(messagebuf," frame=%7d", ndest);
				DrawString(dst,xmsg,2,messagebuf, isYUY2);
				if (nbase == ndest) sprintf(messagebuf," BASE!=%7d", nbase); // CAPITAL letters
				else sprintf(messagebuf," base =%7d", nbase);
				DrawString(dst,xmsg,3,messagebuf, isYUY2);
				sprintf(messagebuf," dx  =%7.2f", dxdif);
				DrawString(dst,xmsg,4,messagebuf, isYUY2);
				sprintf(messagebuf," dy  =%7.2f", dydif);
				DrawString(dst,xmsg,5,messagebuf, isYUY2);
				sprintf(messagebuf," zoom=%7.5f", zoomdif);
				DrawString(dst,xmsg,6,messagebuf, isYUY2);
				sprintf(messagebuf," rot= %7.3f", rotdif);
				DrawString(dst,xmsg,7,messagebuf, isYUY2);
			}


			// This section of code deals with the U and V planes of planar formats (e.g. YV12)

			// result transform for U, V planes (half)
			trUV.dxc = trY.dxc/2;
			trUV.dxx = trY.dxx;
			trUV.dxy = trY.dxy;
			trUV.dyc = trY.dyc/2;
			trUV.dyx = trY.dyx;
			trUV.dyy = trY.dyy;


			if (notfilled == 0) {
				borderUV = -1;  // negative - borders is filled by prev, will not fill by black!
			}
			else
				borderUV = 128; // will fill by black


			//	compensate U plane
			if (subpixel == 2) { // bicubic interpolation
				compensate_plane_bicubic (dstpU,  dst_pitchUV, srcpU,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror*notfilled, borderUV, work2width4356, blur/2);
			}
			else if (subpixel == 1) {  // bilinear interpolation
				compensate_plane_bilinear (dstpU,  dst_pitchUV, srcpU,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror*notfilled, borderUV, work2width4356, blur/2); //
			}
			else {  //subpixel=0, nearest pixel accuracy
				compensate_plane_nearest (dstpU,  dst_pitchUV, srcpU,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror*notfilled, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
			}

			//	compensate V plane
			if (subpixel == 2) { // bicubic interpolation
				compensate_plane_bicubic (dstpV,  dst_pitchUV, srcpV,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror*notfilled, borderUV, work2width4356, blur/2);
			}
			else if (subpixel == 1) {  // bilinear interpolation
				compensate_plane_bilinear (dstpV,  dst_pitchUV, srcpV,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror*notfilled, borderUV, work2width4356, blur/2); //
			}
			else {  //subpixel=0, nearest pixel accuracy
				compensate_plane_nearest (dstpV,  dst_pitchUV, srcpV,  src_pitchUV,  src_widthUV, src_heightUV, trUV, mirror*notfilled, borderUV, work2width4356, blur/2); //  devide dx, dy by 2 for UV plane
			}
		}

	return dst;
}

//****************************************************************************
// This is the function that created the filter, when the filter has been called.
// This can be used for simple parameter checking, so it is possible to create different filters,
// based on the arguments recieved.

AVSValue __cdecl Create_DePanStabilize(AVSValue args, void* user_data, IScriptEnvironment* env) {
    return new DePanStabilize(args[0].AsClip(), // the 0th parameter is the source clip
		 args[1].AsClip(),	// parameter  - motion data clip.
		 (float)args[2].AsFloat(1.0),	// parameter  - cutoff
		 (float)args[3].AsFloat(0.9),	// parameter  - damping  - default changed to 0.9 in v.1.1.4 (was really fixed =0.9 in all prev versions)
		 (float)args[4].AsFloat(1.0),	// parameter  - initial zoom
		 args[5].AsBool(false),	// add adaptive zoom
		 args[6].AsInt(0),		// fill border by prev
		 args[7].AsInt(0),		// fill border by next
		 args[8].AsInt(0),		// fill border by mirror
		 args[9].AsInt(0),		// blur mirror len
		 (float)args[10].AsFloat(60.),	// parameter  - dxmax
		 (float)args[11].AsFloat(30.),	// parameter  - dymax
		 (float)args[12].AsFloat(1.05),	// parameter  - zoommax
		 (float)args[13].AsFloat(1.0),	// parameter  - rotmax
		 args[14].AsInt(2),	// parameter  - subpixel
		 (float)args[15].AsFloat(1.0),	// parameter  - pixaspect
		 args[16].AsInt(0), // fitlast
		 (float)args[17].AsFloat(3.0),	// parameter  - zoom rise time
		 args[18].AsBool(false),	// parameter  - info
		 args[19].AsString(""),  // inputlog filename
		 args[20].AsString(""),  // dx global param
		 args[21].AsString(""),  // dy global param
		 args[22].AsString(""),  // zoom global param
		 args[23].AsString(""),  // rot global param
		 args[24].AsInt(0),	// parameter  - method
		 env);
    // Calls the constructor with the arguments provided.
}


