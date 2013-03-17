/**
 * @file display_stage.c
 * @author nicolas.brulez@parrot.com
 * @date 2012/09/25
 *
 * This stage is a naive example of how to display video using GTK2 + Cairo
 * In a complete application, all GTK handling (gtk main thread + widgets/window creation)
 *  should NOT be handled by the video pipeline (see the Navigation linux example)
 *
 * The window will be resized according to the picture size, and should not be resized bu the user
 *  as we do not handle any gtk event except the expose-event
 *
 * This example is not intended to be a GTK/Cairo tutorial, it is only an example of how to display
 *  the AR.Drone live video feed. The GTK Thread is started here to improve the example readability
 *  (we have all the gtk-related code in one file)
 */

// Self header file
#include "display_stage.h"


//King of the Hill
//TODO: this if to send comand to the drone. You should move this where the drone threads are
#include <ardrone_tool/UI/ardrone_input.h>
#include <cv.h>
#include <highgui.h>

// Funcs pointer definition
const vp_api_stage_funcs_t display_stage_funcs = {
    NULL,
    (vp_api_stage_open_t) display_stage_open,
    (vp_api_stage_transform_t) display_stage_transform,
    (vp_api_stage_close_t) display_stage_close
};

// Extern so we can make the ardrone_tool_exit() function (ardrone_testing_tool.c)
// return TRUE when we close the video window
extern int exit_program;

//King of the Hill variables
//TODO: move this somewhere else, where it's easier to modify them
int MIN_H_HILL_1 = 0;
int MAX_H_HILL_1 = 15;
int MIN_S_HILL = 150;
int MAX_S_HILL = 255;
int MIN_V_HILL = 15;
int MAX_V_HILL = 255;
int pixelRadius;
CvPoint coordinatesOfHillCenter;

//recognize a red ball up until now
void recognizeHills(IplImage* frame){
    
    //-----PHASE 1: DATA SETTING-----//
    
    //Create an HSV image on which we perform transformations and such
    IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
    cvCvtColor(frame, imgHSV, CV_RGB2HSV);
    
    
    //-----PHASE 2: THRESHOLDING AND COLOR RECOGNITION-----//
    
    //Threshold the image (i.e. black and white figure, with white being the object to detect)
    IplImage* imgThresholded = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
    IplImage* imgThresholded2 = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
    
    //To handle color wrap-around, two halves are detected and combined (for red color, otherwise we don't need this)
    cvInRangeS(imgHSV, cvScalar(MIN_H_HILL_1, MIN_S_HILL, MIN_V_HILL, 0), cvScalar(MAX_H_HILL_1, MAX_S_HILL, MAX_V_HILL, 0), imgThresholded); //red-yellow
    cvInRangeS(imgHSV, cvScalar(357, MIN_S_HILL,MIN_V_HILL, 0), cvScalar(360, MAX_S_HILL, MAX_V_HILL, 0), imgThresholded2); //magenta-red
    cvOr(imgThresholded, imgThresholded2, imgThresholded, 0);
    
    
    //-----PHASE 3: SHAPE DETECTION-----//
    
    CvMemStorage* storage = cvCreateMemStorage(0);
    
    //TODO: trying to filter some noise out. This has to be improved!
    cvErode(imgThresholded, imgThresholded, 0, 1);
    cvDilate(imgThresholded, imgThresholded, 0, 2);
    cvSmooth( imgThresholded, imgThresholded, CV_GAUSSIAN, 15, 15, 0, 0 );
    
    //cvHoughCircles(source, circle storage, CV_HOUGH_GRADIENT, resolution, minDist, higher threshold, accumulator threshold, minRadius, maxRadius)
    //minDist = minimum distance between centers of neighborghood detected circles
    //accumulator threshold = at the center detection stage. The smaller it is, the more false circles may be detected.
    //minRadius = minimum radius of the circle to search for
    //maxRadius = max radius of the circles to search for. By default is set to max(image_width, image_height).
    //NOTE: The circles are stored from bigger to smaller.
    //TODO: improve this with live test!!
    CvSeq* circles = cvHoughCircles(imgThresholded, storage, CV_HOUGH_GRADIENT, 2, imgThresholded->height/4, 100, 100, 20, 200);
    
    
    //-----PHASE 4: CIRCLE DRAWING, BIGGEST CIRCLE DATA RETRIVAL AND DIMENSION UPDATING-----//
    
    int i;
    for (i = 0; i < circles->total; i++) {
        
        float* p = (float*)cvGetSeqElem( circles, i );
        
        //I pick only the first circle information because it's the biggest one == nearest
        /*if(i == 0){
            pixelRadius = cvRound(p[2]);
            coordinatesOfHillCenter = cvPoint(cvRound(p[0]), cvRound(p[1]));
        }*/
        
        //cvCircle(frame, center, radius, color, thickness, lineType, shift)
        cvCircle(frame, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(0,255,0), -1, 8, 0); //draw a circle
        cvCircle(frame, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0);
    }

   //-----PHASE 5: FREEING MEMORY-----//
    
    cvReleaseImage(&imgHSV);
    cvReleaseImage(&imgThresholded);
    cvReleaseImage(&imgThresholded2);
    
    cvClearSeq(circles);
    cvClearMemStorage(storage);
    cvReleaseMemStorage(&storage);
}

/*gint startdrone(GtkWidget *widget, gpointer data){
    
    ardrone_tool_set_ui_pad_start(1);
    ardrone_at_set_progress_cmd(0,0,0,0,0);
    
    return C_OK;
}

gint stopdrone(GtkWidget *widget, gpointer data){
    
    ardrone_at_set_progress_cmd(0,0,0,0,0);
    ardrone_tool_set_ui_pad_start(0);
    
    return C_OK;
}*/


C_RESULT display_stage_open(display_stage_cfg_t *cfg){
    return C_OK;
}

C_RESULT display_stage_transform(display_stage_cfg_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out){
    
    IplImage *img = cvCreateImageHeader(cvSize(640, 360), IPL_DEPTH_8U, 3);
    img->imageData = (uint8_t*)in->buffers[in->indexBuffer];
    cvCvtColor(img, img, CV_RGB2BGR);
    
    recognizeHills(img);
    
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0f, 1.0f, 0, 1, CV_AA);
    
    cvPutText(img, "PROVA", cvPoint(30,30), &font, CV_RGB(255,0,0));
    
    //cvNamedWindow("video", CV_WINDOW_AUTOSIZE); //this will show a blank window!!!
    cvShowImage("Video", img);
    
    int c = cvWaitKey(1); //we wait 20ms and if something is pressed during this time, it 'goes' in c
    
    if((char)c == 27) {  //27 is the ASCII code for the escape key (which one is it? the "esc" one!)
        printf("QUTTING!!!!\n");
        // Force ardrone_tool to close
        exit_program = 0;
        // Sometimes, ardrone_tool might not finish properly
        // This happens mainly because a thread is blocked on a syscall
        // in this case, wait 5 seconds then kill the app
        sleep(5);
        exit(0);
    }
    
    cvReleaseImage(&img);

    return C_OK;
}

C_RESULT display_stage_close (display_stage_cfg_t *cfg){
    // Free all allocated memory
    if (NULL != cfg->frameBuffer)
    {
        vp_os_free (cfg->frameBuffer);
        cfg->frameBuffer = NULL;
    }

    return C_OK;
}
