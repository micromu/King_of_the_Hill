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

// GTK/Cairo headers
#include <cairo.h>
#include <gtk/gtk.h>

//#include <../../../../ARDroneLib/Soft/Common/ardrone_api.h>
#include <ardrone_tool/UI/ardrone_input.h>

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

// Boolean to avoid asking redraw of a not yet created / destroyed window
bool_t gtkRunning = FALSE;

// Picture size getter from input buffer size
// This function only works for RGB24 with Cairo (cairo use ARGB also for RGB24, i.e. 4 byte per pixel)
static void getPicSizeFromBufferSize (uint32_t bufSize, uint32_t *width, uint32_t *height){
    
    if (NULL == width || NULL == height){
        return;
    }

    switch (bufSize){
        case 101376: //QCIF > 176*144 *4bpp
            *width = 176;
            *height = 144;
            break;
        case 307200: //QVGA > 320*240 *4bpp
            *width = 320;
            *height = 240;
            break;
        case 921600: //360p > 640*360 *4bpp
            *width = 640;
            *height = 360;
            break;
        case 3686400: //720p > 1280*720 *4bpp 
            *width = 1280;
            *height = 720;
            break;
        default:
            *width = 0;
            *height = 0;
            break;
    }
}

// Get actual frame size (without padding)
void getActualFrameSize(display_stage_cfg_t *cfg, uint32_t *width, uint32_t *height) {
    if (NULL == cfg || NULL == width || NULL == height) {
        return;
    }

    *width = cfg->decoder_info->width;
    *height = cfg->decoder_info->height;
}

// Redraw function, called by GTK each time we ask for a frame redraw
static gboolean on_expose_event(GtkWidget *widget, GdkEventExpose *event, gpointer data){
    
    display_stage_cfg_t *cfg = (display_stage_cfg_t *)data;

    if (3.0 != cfg->bpp) {
        return FALSE;
    }

    uint32_t width = 0, height = 0, stride = 0;
    getPicSizeFromBufferSize (cfg->fbSize, &width, &height);
    stride = cairo_format_stride_for_width(CAIRO_FORMAT_RGB24, width); //this is the right way to know the stride (see Cairo doc)
    
    if (0 == stride) {
        return FALSE;
    }

    uint32_t actual_width = 0, actual_height = 0;
    getActualFrameSize (cfg, &actual_width, &actual_height);
    gtk_window_resize (GTK_WINDOW (widget), actual_width, actual_height);
    
    //create a surface from the video buffer
    cairo_t *cr = gdk_cairo_create(widget->window);
    cairo_surface_t *surface = cairo_image_surface_create_for_data(cfg->frameBuffer, CAIRO_FORMAT_RGB24, width, height, stride);
    cairo_set_source_surface(cr, surface, 0.0, 0.0);
    
    //redraw the frame
    cairo_paint(cr);
    
    //freeing memory
    cairo_surface_destroy(surface);
    cairo_destroy(cr);

    return FALSE;
}

//CUSTOM
#include <cv.h>
#include <highgui.h>
#include <stdio.h>

int MIN_H_HILL_1 = 0;
int MAX_H_HILL_1 = 15;
int MIN_S_HILL = 150;
int MAX_S_HILL = 255;
int MIN_V_HILL = 15;
int MAX_V_HILL = 255;
//ImageData
int buffWidth = 320;
int buffHeight = 240;
int pixelRadius;
CvPoint coordinatesOfHillCenter;

//recognize a red ball up until now
IplImage* recognizeHills(uint8_t* imageBuffer, int reduced_image, int width, int height){
    
    //-----PHASE 1: DATA SETTING-----//
    
    //This is because we have the image buffer readily available
    IplImage* frame = cvCreateImageHeader(cvSize(width,height), IPL_DEPTH_8U,3);
    frame->imageData = imageBuffer;
    
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

    IplImage* RGBA = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 4); //R-G-B-A, 4 channel!!
    cvCvtColor(frame, frame, CV_BGR2RGB); //have to do this because Cairo need RGB and openCV use BGR
    cvCvtColor(frame, RGBA, CV_BGR2BGRA); //here, frame is RGB but I don't care because I just need to add the A after RGB
    
    //-----PHASE 5: FREEING MEMORY-----//
    
    cvReleaseImage(&imgHSV);
    cvReleaseImage(&imgThresholded);
    cvReleaseImage(&imgThresholded2);
    cvReleaseImage(&frame);
    
    cvClearSeq(circles);
    cvClearMemStorage(storage);
    cvReleaseMemStorage(&storage);
    
    return RGBA;
}

gint startdrone(GtkWidget *widget, gpointer data){
    
    ardrone_tool_set_ui_pad_start(1);
    ardrone_at_set_progress_cmd(0,0,0,0,0);
    
    return C_OK;
}

gint stopdrone(GtkWidget *widget, gpointer data){
    
    ardrone_at_set_progress_cmd(0,0,0,0,0);
    ardrone_tool_set_ui_pad_start(0);
    
    return C_OK;
}

/**
 * Main GTK Thread.
 * On an actual application, this thread should be started from your app main thread, and not from a video stage
 * This thread will handle all GTK-related functions
 */
DEFINE_THREAD_ROUTINE(gtk, data)
{
    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    GtkWidget *box;
    GtkWidget *frame;
    GtkWidget *start;
    GtkWidget *stop;

    display_stage_cfg_t *cfg = (display_stage_cfg_t *)data;
    cfg->widget = window;

    g_signal_connect(window, "expose-event", G_CALLBACK(on_expose_event), data);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_window_set_position(GTK_WINDOW (window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(window), 10, 10);
    gtk_window_set_title(GTK_WINDOW(window), "King of the Hill");
    gtk_widget_set_app_paintable(window, TRUE);
    gtk_widget_set_double_buffered(window, FALSE);
    
    //create a frame and adds it to the main window
    frame = gtk_fixed_new();
    gtk_container_add(GTK_CONTAINER(window), frame);
    
    //create a vertical box
    box = gtk_vbox_new(FALSE, 10);
    //gtk_container_add(GTK_CONTAINER(window), box);
    
    //create butttons
    start = gtk_button_new_with_label("start");
    gtk_widget_set_size_request(start, 80, 35);
    
    stop = gtk_button_new_with_label("stop");
    gtk_widget_set_size_request(stop, 80, 35);
    
    //the last parameter is the params to pass to the callback function
    g_signal_connect(G_OBJECT(start), "clicked", G_CALLBACK(startdrone), NULL);
    g_signal_connect(G_OBJECT(stop), "clicked", G_CALLBACK(stopdrone), NULL);
    
    //add the buttons to the vbox
    gtk_box_pack_start(GTK_BOX(box), start, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(box), stop, TRUE, TRUE, 0);
    
    //put the vbox under the cam capture
    gtk_fixed_put(GTK_FIXED(frame), box, 50, 400); //last two params are the position inside the frame
    
    gtk_widget_show_all(window);

    gtkRunning = TRUE;

    gtk_main ();

    gtkRunning = FALSE;

    // Force ardrone_tool to close
    exit_program = 0;

    // Sometimes, ardrone_tool might not finish properly
    // This happens mainly because a thread is blocked on a syscall
    // in this case, wait 5 seconds then kill the app
    sleep (5);
    exit (0);

    return (THREAD_RET)0;
}

C_RESULT display_stage_open (display_stage_cfg_t *cfg)
{
    // Check that we use RGB24
    if (3 != cfg->bpp)
    {
        // If that's not the case, then don't display anything
        cfg->paramsOK = FALSE;
    }
    else
    {
        // Else, start GTK thread and window
        cfg->paramsOK = TRUE;
        cfg->frameBuffer = NULL;
        cfg->fbSize = 0;
        START_THREAD (gtk, cfg);
    }
    return C_OK;
}

C_RESULT display_stage_transform (display_stage_cfg_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    //process the image using openCV
    IplImage *img = recognizeHills((uint8_t*)in->buffers[in->indexBuffer], 1, 640, 360);
   
    // Process only if we are using RGB24
    if (FALSE == cfg->paramsOK) {
        return C_OK;
    }
    
    // Realloc frameBuffer if needed
    // usually needed because I add the A channel to the buffer because of how Cairo handle RGB24 images
    if (img->imageSize != cfg->fbSize){

        cfg->frameBuffer = vp_os_realloc (cfg->frameBuffer, img->imageSize);
        cfg->fbSize = img->imageSize;
    }
    
    //copy the frame processed by openCV to cfg->frameBuffer
    vp_os_memcpy (cfg->frameBuffer, img->imageData, cfg->fbSize);
    

    // Ask GTK to redraw the window
    uint32_t width = 0, height = 0;
    getPicSizeFromBufferSize (img->imageSize, &width, &height);
    if (TRUE == gtkRunning)
    {
        //this fire an expose event, causing the on_expose_event() function to be called (see signals in user_interface thread)
        gtk_widget_queue_draw_area (cfg->widget, 0, 0, width, height);
    }
    
    cvReleaseImage(&img); //TODO: però non risolve il problema....

    // Tell the pipeline that we don't have any output
    out->size = 0;

    return C_OK;
}

C_RESULT display_stage_close (display_stage_cfg_t *cfg)
{
    // Free all allocated memory
    if (NULL != cfg->frameBuffer)
    {
        vp_os_free (cfg->frameBuffer);
        cfg->frameBuffer = NULL;
    }

    return C_OK;
}
