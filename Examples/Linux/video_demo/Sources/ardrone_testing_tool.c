/**
 * @file ardrone_testing_tool.c
 * @author nicolas.brulez@parrot.com
 * @date 2012/09/04
 *
 * Example of AR.Drone Live Video Feed using the AR.Drone SDK 2.0
 * This example works for both AR.Drone 1 and 2
 *
 * Show example of pre-decoding processing (record encoded h.264 frames)
 * and post-decoding processing (display decoded frames)
 *
 * Possible args :
 *  -eFileName : Record encoded video to FileName
 *               - If AR.Drone 2.0 is used, and filename ends with ".h264"
 *               it can be read with any standard video player that supports
 *               raw h264 (ffplay, mplayer ...)
 *               - AR.Drone 1 video must be transcoded before use. See iOS
 *               AR.FreeFlight app for an example
 *         NOTE : if -eNAME arg is not present, encoded video will not be recorded
 *         NOTE : This is NOT the equivalent of the official record function for AR.Drone2 !
 *
 *  -b : use bottom camera instead of frontal camera
 *
 *  -c : use alternative video codec
 *       - For AR.Drone 2 -> 720p instead of 360p (both h.264)
 *       - For AR.Drone 1 -> VLIB instead of P264
 *
 * NOTE : Frames will be displayed only if out_picture->format is set to PIX_FMT_RGB565
 *
 * Display examlpe uses GTK2 + Cairo.
 */

// Generic includes
#include <ardrone_api.h>
#include <signal.h>

// ARDrone Tool includes
#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/ardrone_tool_configuration.h>
#include <ardrone_tool/ardrone_version.h>
#include <ardrone_tool/Video/video_stage.h>
#include <ardrone_tool/Video/video_recorder_pipeline.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>

// App includes
#include <Video/pre_stage.h>
#include <Video/display_stage.h>

// GTK includes
#include <gtk/gtk.h>

int exit_program = 1;

pre_stage_cfg_t precfg;
display_stage_cfg_t dispCfg;

codec_type_t drone1Codec = P264_CODEC;
codec_type_t drone2Codec = H264_360P_CODEC;
ZAP_VIDEO_CHANNEL videoChannel = ZAP_CHANNEL_HORI;

#define FILENAMESIZE (256)
char encodedFileName[FILENAMESIZE] = {0};

void controlCHandler (int signal)
{
    // Flush all streams before terminating
    fflush (NULL);
    usleep (200000); // Wait 200 msec to be sure that flush occured
    printf ("\nAll files were flushed\n");
    exit (0);
}

/**
 * This example shows how to get the AR.Drone Live Video feed
 */
int main (int argc, char *argv[])
{
    signal (SIGABRT, &controlCHandler);
    signal (SIGTERM, &controlCHandler);
    signal (SIGINT, &controlCHandler);
    int prevargc = argc;
    char **prevargv = argv;

    int index = 0;
    for (index = 1; index < argc; index++)
    {
        if ('-' == argv[index][0] &&
            'e' == argv[index][1])
        {
            char *fullname = argv[index];
            char *name = &fullname[2];
            strncpy (encodedFileName, name, FILENAMESIZE);
        }

        if ('-' == argv[index][0] &&
            'c' == argv[index][1])
        {
            drone1Codec = UVLC_CODEC;
            drone2Codec = H264_720P_CODEC;
        }

        if ('-' == argv[index][0] &&
            'b' == argv[index][1])
        {
            videoChannel = ZAP_CHANNEL_VERT;
        }
    }

    gtk_init (&prevargc, &prevargv);

    return ardrone_tool_main (prevargc, prevargv);
}

C_RESULT ardrone_tool_init_custom (void)
{
    /**
     * Set application default configuration
     *
     * In this example, we use the AR.FreeFlight configuration :
     * - Demo navdata rate (15Hz)
     * - Useful additionnal navdata packets enabled (detection, games, video record, wifi quality estimation ...)
     * - Adaptive video enabled (bitrate_ctrl_mode) -> video bitrate will change according to the available bandwidth
     */
    ardrone_application_default_config.navdata_demo = TRUE;
    ardrone_application_default_config.navdata_options = (NAVDATA_OPTION_MASK(NAVDATA_DEMO_TAG) | NAVDATA_OPTION_MASK(NAVDATA_VISION_DETECT_TAG) | NAVDATA_OPTION_MASK(NAVDATA_GAMES_TAG) | NAVDATA_OPTION_MASK(NAVDATA_MAGNETO_TAG) | NAVDATA_OPTION_MASK(NAVDATA_HDVIDEO_STREAM_TAG) | NAVDATA_OPTION_MASK(NAVDATA_WIFI_TAG));
    if (IS_ARDRONE2)
    {
        ardrone_application_default_config.video_codec = drone2Codec;
    }
    else
    {
        ardrone_application_default_config.video_codec = drone1Codec;
    }
    ardrone_application_default_config.video_channel = videoChannel;
    ardrone_application_default_config.bitrate_ctrl_mode = 1;

    /**
     * Define the number of video stages we'll add before/after decoding
     */
#define EXAMPLE_PRE_STAGES 1
#define EXAMPLE_POST_STAGES 1

    /**
     * Allocate useful structures :
     * - index counter
     * - thread param structure and its substructures
     */
    uint8_t stages_index = 0;

    specific_parameters_t *params = (specific_parameters_t *)vp_os_calloc (1, sizeof (specific_parameters_t));
    specific_stages_t *example_pre_stages = (specific_stages_t *)vp_os_calloc (1, sizeof (specific_stages_t));
    specific_stages_t *example_post_stages = (specific_stages_t *)vp_os_calloc (1, sizeof (specific_stages_t));
    vp_api_picture_t *in_picture = (vp_api_picture_t *)vp_os_calloc (1, sizeof (vp_api_picture_t));
    vp_api_picture_t *out_picture = (vp_api_picture_t *)vp_os_calloc (1, sizeof (vp_api_picture_t));

    /**
     * Fill the vp_api_pictures used for video decodig
     * --> out_picture->format is mandatory for AR.Drone 1 and 2. Other lines are only necessary for AR.Drone 1 video decoding
     */
    in_picture->width = 640; // Drone 1 only : Must be greater than the drone 1 picture size (320)
    in_picture->height = 360; // Drone 1 only : Must be greater that the drone 1 picture size (240)

    out_picture->framerate = 20; // Drone 1 only, must be equal to drone target FPS
    out_picture->format = PIX_FMT_RGB24; // MANDATORY ! Only RGB24, RGB565 are supported
    out_picture->width = in_picture->width;
    out_picture->height = in_picture->height;

    // Alloc Y, CB, CR bufs according to target format
    uint32_t bpp = 0;
    switch (out_picture->format)
    {
    case PIX_FMT_RGB24:
        // One buffer, three bytes per pixel
        bpp = 3;
        out_picture->y_buf = vp_os_malloc ( out_picture->width * out_picture->height * bpp );
        out_picture->cr_buf = NULL;
        out_picture->cb_buf = NULL;
        out_picture->y_line_size = out_picture->width * bpp;
        out_picture->cb_line_size = 0;
        out_picture->cr_line_size = 0;
        break;
    case PIX_FMT_RGB565:
        // One buffer, two bytes per pixel
        bpp = 2;
        out_picture->y_buf = vp_os_malloc ( out_picture->width * out_picture->height * bpp );
        out_picture->cr_buf = NULL;
        out_picture->cb_buf = NULL;
        out_picture->y_line_size = out_picture->width * bpp;
        out_picture->cb_line_size = 0;
        out_picture->cr_line_size = 0;
        break;
    default:
        fprintf (stderr, "Wrong video format, must be either PIX_FMT_RGB565 or PIX_FMT_RGB24\n");
        exit (-1);
        break;
    }

    /**
     * Allocate the stage lists
     *
     * - "pre" stages are called before video decoding is done
     *  -> A pre stage get the encoded video frame (including PaVE header for AR.Drone 2 frames) as input
     *  -> A pre stage MUST NOT modify these data, and MUST pass it to the next stage
     * - Typical "pre" stage : Encoded video recording for AR.Drone 1 (recording for AR.Drone 2 is handled differently)
     *
     * - "post" stages are called after video decoding
     *  -> The first post stage will get the decoded video frame as its input
     *   --> Video frame format depend on out_picture->format value (RGB24 / RGB565)
     *  -> A post stage CAN modify the data, as ardrone_tool won't process it afterwards
     *  -> All following post stages will use the output of the previous stage as their inputs
     * - Typical "post" stage : Display the decoded frame
     */
    example_pre_stages->stages_list = (vp_api_io_stage_t *)vp_os_calloc (EXAMPLE_PRE_STAGES, sizeof (vp_api_io_stage_t));
    example_post_stages->stages_list = (vp_api_io_stage_t *)vp_os_calloc (EXAMPLE_POST_STAGES, sizeof (vp_api_io_stage_t));

    /**
     * Fill the PRE stage list
     * - name and type are debug infos only
     * - cfg is the pointer passed as "cfg" in all the stages calls
     * - funcs is the pointer to the stage functions
     */
    stages_index = 0;

    vp_os_memset (&precfg, 0, sizeof (pre_stage_cfg_t));
    strncpy (precfg.outputName, encodedFileName, 255);

    example_pre_stages->stages_list[stages_index].name = "Encoded Dumper"; // Debug info
    example_pre_stages->stages_list[stages_index].type = VP_API_FILTER_DECODER; // Debug info
    example_pre_stages->stages_list[stages_index].cfg  = &precfg;
    example_pre_stages->stages_list[stages_index++].funcs  = pre_stage_funcs;

    example_pre_stages->length = stages_index;

    /**
     * Fill the POST stage list
     * - name and type are debug infos only
     * - cfg is the pointer passed as "cfg" in all the stages calls
     * - funcs is the pointer to the stage functions
     */
    stages_index = 0;

    vp_os_memset (&dispCfg, 0, sizeof (display_stage_cfg_t));
    dispCfg.bpp = bpp;
    dispCfg.decoder_info = in_picture;

    example_post_stages->stages_list[stages_index].name = "Decoded display"; // Debug info
    example_post_stages->stages_list[stages_index].type = VP_API_OUTPUT_SDL; // Debug info
    example_post_stages->stages_list[stages_index].cfg  = &dispCfg;
    example_post_stages->stages_list[stages_index++].funcs  = display_stage_funcs;

    example_post_stages->length = stages_index;

    /**
     * Fill thread params for the ardrone_tool video thread
     *  - in_pic / out_pic are reference to our in_picture / out_picture
     *  - pre/post stages lists are references to our stages lists
     *  - needSetPriority and priority are used to control the video thread priority
     *   -> if needSetPriority is set to 1, the thread will try to set its priority to "priority"
     *   -> if needSetPriority is set to 0, the thread will keep its default priority (best on PC)
     */
    params->in_pic = in_picture;
    params->out_pic = out_picture;
    params->pre_processing_stages_list  = example_pre_stages;
    params->post_processing_stages_list = example_post_stages;
    params->needSetPriority = 0;
    params->priority = 0;

    /**
     * Start the video thread (and the video recorder thread for AR.Drone 2)
     */
    START_THREAD(video_stage, params);
    START_THREAD(wiimote, NULL);
    video_stage_init();
    /*if (2 <= ARDRONE_VERSION ())
    {
        START_THREAD (video_recorder, NULL);
        video_recorder_init ();
    }*/

    video_stage_resume_thread ();

    return C_OK;
}

C_RESULT ardrone_tool_shutdown_custom ()
{
    video_stage_resume_thread(); //Resume thread to kill it !
    JOIN_THREAD(video_stage);
    JOIN_THREAD(wiimote);
    /*if (2 <= ARDRONE_VERSION ())
    {
        video_recorder_resume_thread ();
        JOIN_THREAD (video_recorder);
    }*/

    return C_OK;
}

bool_t ardrone_tool_exit ()
{
    return exit_program == 0;
}

#include "global_variables.h"
#include <cwiid.h>

//NEW THREADS ADDED BY ME
DEFINE_THREAD_ROUTINE(drone_logic, data){
    //the game is active from start, but the logic will start only when a match is active
    while(game_active){
        if(match){
            
            //CHASING - N.B. hill have higher priority than enemy
            if(hill_in_sight){
                //move toward the hill
                if((hill_distance > hill_min_distance) && (hill_distance < hill_max_distance)){
                    //TODO: collina a distanza ragionevole. avvicinati
                    
                    //hover over the hill
                } else if(hill_distance < hill_min_distance) {
                    //TODO: inizia la procedura di riconoscimento collina (cambia cam, hover...)
                    //set a variable to tell the score logic to add one for the drone
                }
            } else if(enemy_in_sight){
                if(enemy_distance < enemy_min_distance){
                    //TODO: too close!! back up
                    
                    //TODO: implements a good algorithm that choose when to shoot and when not
                    // e.g. after some time (or randomly), the drone should stop shooting and/or chasing the enemy
                } else if((enemy_distance > enemy_min_distance) && (enemy_distance < enemy_shooting_distance)){
                    //TODO: shoot!!
                    
                } else if((enemy_distance > enemy_shooting_distance) && (enemy_distance < enemy_max_distance)){
                    //TODO: move toward the enemy
                }
            } else {
                //TODO: nothing in sight: hover and start the locator algorithm
            }
            
            //HIT
            if(drone_wounded){
                //TODO: make the drone move as if it was being shot
                //maybe this should be moved in the flying thread
            }
            
            //MATCH OVER
        } else {
            //TODO: land the drone
        }
    }
}

DEFINE_THREAD_ROUTINE(fly_control, data){
    //TODO: I REALLY NEED THIS?
}

DEFINE_THREAD_ROUTINE(wiimote, data){
    int ammunitions = number_of_ammo;
    
    static bdaddr_t bdaddr = {0};
    static cwiid_wiimote_t *wiimote = NULL;
    union cwiid_mesg *msg = NULL;
    struct timespec timestamp;
    
    int wiimote_connected = 0;
    
    int drone_led = 0;
    int shoot_button = 0;
    int number_of_led = 0;
    int recharging_led = 0;
    int recharging_button = 0;
    int i = 0;
    int j = 0;
    int msg_count = 0;
    
    while(game_active){
        
        shoot_button = 0;
        recharging_button = 0;
        
        number_of_led = 0;
        recharging_led = 0;
        drone_led = 0;
        
        
        //CONNECT TO THE WIIMOTE
        if(wiimote_connected == 0) {
            if( ! (wiimote = cwiid_open(&bdaddr, CWIID_FLAG_MESG_IFC)) ) {
                printf("Unable to connect to wiimote\n");
            } else {
                wiimote_connected = 1;
                printf("Wiimote found\n");
                cwiid_command(wiimote, CWIID_CMD_LED, CWIID_LED1_ON|CWIID_LED2_ON|CWIID_LED3_ON|CWIID_LED4_ON);
                cwiid_command(wiimote, CWIID_CMD_RPT_MODE, CWIID_RPT_IR|CWIID_RPT_ACC|CWIID_RPT_BTN);
            }
            
        //ALREADY CONNECTED
        } else {
            //--- GET INPUTS FROM THE WIIMOTE ---//
            
            //get messages (blocking)
            cwiid_get_mesg(wiimote, &msg_count, &msg, &timestamp);
            
            //scan the messages for the event "pression of shoot_button" or "pression of recharging_button"
            //and to count the number of IR leds found (4 leds == recharge, other == drone //TODO: define the number of leds for the drone)
            for(i = 0; i < msg_count; i++){
                
                if(msg[i].type == CWIID_MESG_BTN){
                    if(msg[i].btn_mesg.buttons == CWIID_BTN_B){
                        shoot_button = 1;
                        printf("SHOOT\n");
                    }
                    
                    if(msg[i].btn_mesg.buttons == CWIID_BTN_A){
                        recharging_button = 1;
                        printf("BUTTON A\n");
                    }
                }
                
                //NOTE: the wiimote find also hot source of light and the sun!!
                if(msg[i].type == CWIID_MESG_IR){ 
                    for(j = 0; j < CWIID_IR_SRC_COUNT; j++){
                        if(msg[i].ir_mesg.src[j].valid != 0){
                            number_of_led++;
                        }
                    }
                    if(number_of_led == 4){
                        recharging_led = 1;
                        printf("FOUR LED\n");
                    } else if(number_of_led > 0) {
                        //TODO: I can use 2 leds to identify the drone? should I?
                        drone_led = 1;
                        printf("LEDS\n");
                    }
                }
            }
            if(ammunitions > 0 /*TODO: add -> && trigger_pressed*/){
                
                //TODO: -1 ammo
                //sound and/or vibration
                
                if(drone_in_sight){
                    //TODO: drone wounded, notify the score logic or just make the drone stop? you have to decide!
                }
                
            } else {
                //TODO: you have to recharge
                //check if recharge site in sight, check if button pressed, wait tot seconds, reset ammo counter
            }
        }
    } 
}

DEFINE_THREAD_ROUTINE(score_logic, data){
    
}


/**
 * Declare Thread / Navdata tables
 */

// Declare gtk thread and include it in the thread table
//  This is needed because the display_stage.c file can't access this table
PROTO_THREAD_ROUTINE(gtk, data);

BEGIN_THREAD_TABLE
THREAD_TABLE_ENTRY(video_stage, 20)
THREAD_TABLE_ENTRY(video_recorder, 20)
THREAD_TABLE_ENTRY(navdata_update, 20)
THREAD_TABLE_ENTRY(ardrone_control, 20)
THREAD_TABLE_ENTRY(gtk, 20)
THREAD_TABLE_ENTRY(drone_logic, 20)
THREAD_TABLE_ENTRY(fly_control, 20)
THREAD_TABLE_ENTRY(wiimote, 20)
THREAD_TABLE_ENTRY(score_logic, 20)
END_THREAD_TABLE

BEGIN_NAVDATA_HANDLER_TABLE
END_NAVDATA_HANDLER_TABLE
