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

//King of the Hill
#include "global_variables.h"
#include <cwiid.h>
//ardrone_api is needed for led animation
#include <Soft/Common/ardrone_api.h>
//ardrone_input is needed for drone movement
#include <ardrone_tool/UI/ardrone_input.h>

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
        
        if ('-' == argv[index][0] &&
            'd' == argv[index][1])
        {
            debugging = 1;
        }
    }

    return ardrone_tool_main(prevargc, prevargv);
}

C_RESULT ardrone_tool_init_custom(void)
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
    //use this -> NAVDATA_OPTION_FULL_MASK
    //or coment the line below if detection doesn't work
    ardrone_application_default_config.navdata_options = NAVDATA_OPTION_FULL_MASK;//(NAVDATA_OPTION_MASK(NAVDATA_DEMO_TAG) | NAVDATA_OPTION_MASK(NAVDATA_VISION_DETECT_TAG) | NAVDATA_OPTION_MASK(NAVDATA_GAMES_TAG) | NAVDATA_OPTION_MASK(NAVDATA_MAGNETO_TAG) | NAVDATA_OPTION_MASK(NAVDATA_HDVIDEO_STREAM_TAG) | NAVDATA_OPTION_MASK(NAVDATA_WIFI_TAG));
    if (IS_ARDRONE2){
        ardrone_application_default_config.video_codec = drone2Codec;
    } else {
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
    
    
    //set the tag detection
    ENEMY_COLORS_TYPE enemyColors = ARDRONE_DETECTION_COLOR_ORANGE_BLUE;
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT (enemy_colors, &enemyColors, NULL);
    
    CAD_TYPE detectType = CAD_TYPE_MULTIPLE_DETECTION_MODE;
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detect_type, &detectType, NULL);
    
    uint32_t detectH = TAG_TYPE_MASK (TAG_TYPE_SHELL_TAG_V2);
    ARDRONE_TOOL_CONFIGURATION_ADDEVENT (detections_select_h, &detectH, NULL);
    

    /**
     * Start the video thread (and the video recorder thread for AR.Drone 2)
     */
    START_THREAD(video_stage, params);
    START_THREAD (video_recorder, NULL);
    video_stage_init();
    if (2 <= ARDRONE_VERSION ())
    {
        START_THREAD (video_recorder, NULL);
        video_recorder_init ();
    }
    video_stage_resume_thread();
    
    //King of the Hill threads
    START_THREAD(wiimote_logic, NULL);
    START_THREAD(drone_logic, NULL);
    START_THREAD(score_logic, NULL);
    
    return C_OK;
}

C_RESULT ardrone_tool_shutdown_custom (){
    
    //King of the Hill threads
    JOIN_THREAD(wiimote_logic);
    JOIN_THREAD(drone_logic);
    JOIN_THREAD(score_logic);
    
    video_stage_resume_thread(); //Resume thread to kill it !
    JOIN_THREAD(video_stage);
    if (2 <= ARDRONE_VERSION ())
    {
        video_recorder_resume_thread ();
        JOIN_THREAD (video_recorder);
    }

    return C_OK;
}

bool_t ardrone_tool_exit (){
    
    return exit_program == 0;
}

//NEW THREADS ADDED BY ME
DEFINE_THREAD_ROUTINE(drone_logic, data){
    //the game is active from start, but the logic will start only when a match is active
    
    //struct timespec shot_rumble_time;
    //shot_rumble_time.tv_sec = 1;
    //shot_rumble_time.tv_nsec = 0000000;
    int emptiness_counter = 0;
    int shooting_counter = 0;
    
    int hovering = 0; //0 hover, 1 move
    float phi = 0.0; //left/right angle. Between [-1.0,+1.0], with negatives being leftward movement
    float theta = 0.0; //front/back angle. Between [-1.0, +1.0], with negatives being frontward movement
    float gaz = 0.0; //vertical speed. Between [-1.0, +1.0]
    float yaw = 0.0; //Angular speed. Between [-1.0, +1.0]
    
    while(game_active){
        if(match_active){
            
            if(takeoff){
                //TODO:Uncomment this when you are ready to make the drone move autonomously
                //ardrone_tool_set_ui_pad_start(1);
                //ardrone_at_set_progress_cmd(0,0.0,0.0,0.0,0.0);
                takeoff = 0;
            }
            
            //--- CHASING ---// //NOTE: Hill have higher priority than enemy
            if(hill_in_sight){
                emptiness_counter = 0;
                shooting_counter = 0;
                
                //move toward the hill
                if((hill_distance > HILL_MIN_DISTANCE) && (hill_distance < HILL_MAX_DISTANCE)){
                    
                    printf("MOVING TOWARD THE HILL\n");
                    
                    hovering = 1;
                    phi = 0.0;
                    gaz = 0.0;
                    
                    //--- SET THE YAW ---//
                    //This is to correct the direction of the drone
                    if(abs(hill_offset_from_center) > ERROR_FROM_CENTER_FOR_HILL){
                        //TODO: find the right multiplier for yaw and discover which way the drone turn
                        yaw = (hill_offset_from_center) * YAW_COEFF; //YAW_COEFF = 0.007
                        //yaw has to be between -1.0 and +1.0
                        if(yaw > 1.0) {
                            yaw = 1.0;
                        } else if(yaw < -1.0){
                            yaw = -1.0;
                        }
                    }
                    
                    //--- SET THE APPROACHING SPEED ---//
                    //The closer the drone is to the hill, the slower it goes
                    //TODO: find the right theta_coeff
                    theta = -1*((hill_distance) / THETA_COEFF); //Need to be negative to move forward
                     //with -1.0 being frontward movement (drone bend frontward)
                    if(theta > 0.0) { //TODO: this need to be better defined.
                        theta = 0.0;
                    } else if(theta < -1.0){ 
                        theta = -1.0;
                    }
                    
                    //TODO: this has to be here or to be moved somewhere else? maybe right after this big if?
                    //ardrone_at_set_progress_cmd(hovering,phi,theta,gaz,yaw);
                    
                //hover over the hill
                } else if(hill_distance < HILL_MIN_DISTANCE) {
                    //ardrone_at_set_progress_cmd(0,0,0,0,0); //to hover
                    
                    printf("HOVERING ON TOP OF THE HILL\n");
                    
                    hovering = 0;
                    phi = 0.0;
                    theta = 0.0;
                    gaz = 0.0;
                    yaw = 0.0;
                    
                    //TODO: if you are close enough, you have to switch the cam and then inizialize 
                    //the recognition procedure. (wait tot secs)
                    vp_os_mutex_lock(&drone_score_mutex);
                        drone_add_score = 1;
                    vp_os_mutex_unlock(&drone_score_mutex);
                    //TODO: here, you switch the camera back
                }
                
            } else if(enemy_in_sight){
                emptiness_counter = 0;
                
                //back up! Too close to the enemy (we don't want to phisically hit the human player!)
                if(enemy_distance < ENEMY_MIN_DISTANCE){
                    shooting_counter = 0;
                    
                    hovering = 1;
                    theta = 1.0; //Move backward at maximum speed
                    phi = 0.0;
                    gaz = 0.0;
                    yaw = 0.0;
                    
                    printf("BACKING UP FROM THE ENEMY\n");
                    
                } else if((enemy_distance > ENEMY_MIN_DISTANCE) && (enemy_distance < ENEMY_SHOOTING_DISTANCE)){
                    
                    shooting_counter++;
                    
                    if(shooting_counter > 5){
                        
                        //TODO: make the drone move
                        
                    } else {
                        
                        //TODO: shoot!!
                        //make animation.
                        
                        if(enemy_offset_from_center < ERROR_FROM_CENTER_FOR_ENEMY){
                            vp_os_mutex_lock(&enemy_score_mutex);
                            enemy_lose_score = 1;
                            vp_os_mutex_lock(&enemy_score_mutex);
                        }
                        
                        printf("SHOOTING!!!!!!!!\n");
                        
                        hovering = 0;
                        phi = 0.0;
                        theta = 0.0;
                        gaz = 0.0;
                        
                        //--- SET THE YAW ---//
                        //This is to correct the direction of the drone
                        if(abs(enemy_offset_from_center) > ERROR_FROM_CENTER_FOR_ENEMY){
                            //TODO: find the right multiplier for yaw and discover wich way the drone turn
                            yaw = (enemy_offset_from_center) * YAW_COEFF; //YAW_COEFF = 0.007
                            //yaw has to be between -1.0 and +1.0
                            if(yaw > 1.0) {
                                yaw = 1.0;
                            } else if(yaw < -1.0){
                                yaw = -1.0;
                            }
                        }
                    }
                
                } else if((enemy_distance > ENEMY_SHOOTING_DISTANCE) && (enemy_distance < ENEMY_MAX_DISTANCE)){
                    shooting_counter = 0;
                    
                    //TODO: in this case I may want to just make the drone search for hills
                    //TODO: it may escape, turning itself so the led won't face the enemy anymore
                    
                    //--- SET THE YAW ---//
                    //This is to correct the direction of the drone
                    if(abs(enemy_offset_from_center) > ERROR_FROM_CENTER_FOR_ENEMY){
                        //TODO: find the right multiplier for yaw and discover wich way the drone turn
                        //TODO: in this case, I won't to make the drone move away from the enemy, so the algorithm below 
                        //has to be changed
                        yaw = (enemy_offset_from_center) * YAW_COEFF; //YAW_COEFF = 0.007
                        //yaw has to be between -1.0 and +1.0
                        if(yaw > 1.0) {
                            yaw = 1.0;
                        } else if(yaw < -1.0){
                            yaw = -1.0;
                        }
                    }
                }
                
                //After calculation, make the drone move
                //ardrone_at_set_progress_cmd(hovering,phi,theta,gaz,yaw);
            
            //NOTHING IN SIGHT
            } else {
                //If nothing is in sight I set a counter that increment every time I pass directly from here 
                //and make the drone rotate around itself. After tot passages with nothing in sight I take measure to land the drone.
                if(emptiness_counter == 0){
                    //TODO: set up everything for the algorithm to start
                    emptiness_counter = 1;
                    //yaw = something != to zero;
                    //everything else if zero (I don't know about hovering... problably should be 1 here)
                    //ardrone_at_set_progress_cmd(hovering,phi,theta,gaz,yaw);
                } else {
                    emptiness_counter++;
                    //TODO: should I set a little sleep (less than 1sec) here or in the condition below?
                    if(emptiness_counter > 10){
                        //TODO:land the drone and make the game stop
                        //so set everything that should to zero
                    }
                }
            }
            
            //NOTE: I don't know why, but if I add this printf the condition below works
            printf("%d",drone_wounded);
            
            //HIT
            if(drone_wounded){
                //TODO: make the drone move as if it was being shot
                //maybe this should be moved in the flying thread
                vp_os_mutex_lock(&drone_wound_mutex);
                    drone_wounded = 0;
                vp_os_mutex_unlock(&drone_wound_mutex);
                
                //TODO: you can choose between this animation, defined in Soft/Common/config.h
                /*ARDRONE_ANIM_PHI_M30_DEG= 0,
                 A RDRONE_ANIM_PHI_30_DE*G,
                 ARDRONE_ANIM_THETA_M30_DEG,
                 ARDRONE_ANIM_THETA_30_DEG,
                 ARDRONE_ANIM_THETA_20DEG_YAW_200DEG,
                 ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG,
                 ARDRONE_ANIM_TURNAROUND,
                 ARDRONE_ANIM_TURNAROUND_GODOWN,
                 ARDRONE_ANIM_YAW_SHAKE,
                 ARDRONE_ANIM_YAW_DANCE,
                 ARDRONE_ANIM_PHI_DANCE,
                 ARDRONE_ANIM_THETA_DANCE,
                 ARDRONE_ANIM_VZ_DANCE,
                 ARDRONE_ANIM_WAVE,
                 ARDRONE_ANIM_PHI_THETA_MIXED,
                 ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED,
                 ARDRONE_ANIM_FLIP_AHEAD,
                 ARDRONE_ANIM_FLIP_BEHIND,
                 ARDRONE_ANIM_FLIP_LEFT,
                 ARDRONE_ANIM_FLIP_RIGHT,
                 ARDRONE_NB_ANIM_MAYDAY*/
                //anim_mayday_t param;
                //ARDRONE_TOOL_CONFIGURATION_ADDEVENT (flight_anim, param, myCallback);
                ardrone_at_set_led_animation(BLINK_GREEN_RED, 0.25, 4);
                //TODO: freeze the drone for some time, also?
                //nanosleep(&shot_rumble_time, NULL);
            }
            
        //MATCH OVER
        } else {
            //land the drone
            ardrone_at_set_progress_cmd(0,0.0,0.0,0.0,0.0);
            ardrone_tool_set_ui_pad_start(0);
            //TODO: I may need some sleep time to make the drone land before closing everyting
        }
    }
    
    return C_OK;
}

DEFINE_THREAD_ROUTINE(wiimote_logic, data){
    
    //wiimote connection variables
    static bdaddr_t bdaddr = {{0}};
    static cwiid_wiimote_t *wiimote = NULL;
    union cwiid_mesg *msg = NULL;
    struct timespec timestamp;
    
    int wiimote_connected = 0;
    
    int drone_in_sight = 0;
    int trigger_button = 0;
    int number_of_led = 0;
    int recharger_in_sight = 0;
    int recharging_button = 0;
    
    int bullets = magazine_capacity;
    
    int i = 0;
    int j = 0;
    int msg_count = 0;
    
    struct timespec shot_rumble_time;
    shot_rumble_time.tv_sec = 1;
    shot_rumble_time.tv_nsec = 0000000;

    struct timespec recharging_time;
    recharging_time.tv_sec = 10;
    recharging_time.tv_nsec = 0000000;
    
    while(game_active){
        
        //CONNECT TO THE WIIMOTE
        if(wiimote_connected == 0) {
            if( ! (wiimote = cwiid_open(&bdaddr, CWIID_FLAG_MESG_IFC)) ) {
                printf("Unable to connect to wiimote\n");
            } else {
                wiimote_connected = 1;
                printf("Wiimote found\n");
                cwiid_command(wiimote, CWIID_CMD_LED, CWIID_LED1_ON|CWIID_LED2_ON|CWIID_LED3_ON|CWIID_LED4_ON);
                cwiid_command(wiimote, CWIID_CMD_RPT_MODE, CWIID_RPT_IR|CWIID_RPT_BTN);
            }
            
        //ALREADY CONNECTED
        } else {
            if(match_active){
                
                //--- RESET VARIABLES ---//
                number_of_led = 0;
                drone_in_sight = 0;
                recharger_in_sight = 0;
                trigger_button = 0;
                msg_count = 0;
                
                //--- GET INPUTS FROM THE WIIMOTE ---//
                
                //get messages (blocking)
                cwiid_get_mesg(wiimote, &msg_count, &msg, &timestamp);
                
                //scan the messages for the event "pression of trigger_button" or "pression of recharging_button"
                //and to count the number of IR leds found
                //NOTE: the wiimote find false positive (sometimes 1led == 4leds :O)
                //NOTE: the wiimote is REALLY sensitive to sun light
                for(i = 0; i < msg_count; i++){
                    
                    if(msg[i].type == CWIID_MESG_BTN){
                        
                        //is button B pressed?
                        if(msg[i].btn_mesg.buttons == CWIID_BTN_B){
                            trigger_button = 1;
                            printf("SHOOT\n");
                        } else {
                            trigger_button = 0;
                        }
                        
                        //is button A pressed?
                        if(msg[i].btn_mesg.buttons == CWIID_BTN_A){
                            recharging_button = 1;
                            printf("BUTTON A\n");
                        } else {
                            recharging_button = 0;
                        }
                        
                        //TODO: this is here in case the ar.drone stop sending video update
                        if(msg[i].btn_mesg.buttons == CWIID_BTN_HOME){
                            printf("The program will shutdown...\n");
                            
                            match_active = 0; //This tell the drone_logic thread to land the drone
                            game_active = 0; //This make all the threads exit the while loop
                            
                            exit_program = 0;  // Force ardrone_tool to close
                            // Sometimes, ardrone_tool might not finish properly. 
                            //This happens mainly because a thread is blocked on a syscall, in this case, wait 5 seconds then kill the app
                            sleep(5);
                            exit(0);
                        }
                        
                    }
                    
                    //are there leds?
                    if(msg[i].type == CWIID_MESG_IR){ 
                        for(j = 0; j < CWIID_IR_SRC_COUNT; j++){
                            if(msg[i].ir_mesg.src[j].valid != 0){
                                number_of_led++;
                            }
                        }
                        
                        if(number_of_led > 0){
                            printf("LEDS\n");
                            drone_in_sight = 1;
                        } else {
                            drone_in_sight = 0;
                        }
                    }
                }
                
                //--- WIIMOTE LOGIC ---//
                //SHOOTING
                if((bullets > 0) && trigger_button){
                    
                    bullets--;
                    printf("lost one bullet\n");
                    
                    //haptic feedback
                    cwiid_command(wiimote, CWIID_CMD_RUMBLE, 1);
                    nanosleep(&shot_rumble_time, NULL);
                    cwiid_command(wiimote, CWIID_CMD_RUMBLE, 0);
                    
                    if(drone_in_sight){
                        
                        vp_os_mutex_lock(&drone_score_mutex);
                            drone_lose_score = 1;
                        vp_os_mutex_unlock(&drone_score_mutex);
                        
                        printf("DRONE HIT\n");
                        
                    } else {
                        printf("DRONE MISSED!!\n");
                    }
                    
                    //This is to limit the frequency of shooting (i.e. the gun need to load)
                    nanosleep(&shot_rumble_time, NULL);
                    
                //you can recharge only if you don't have bullets any more
                } else if(bullets < 1){
                    if(recharging_button){
                        //TODO: How should I let the enemy know that the wiimote is full again?
                        nanosleep(&recharging_time, NULL);
                        bullets = magazine_capacity;
                    }
                }
            }
        }
    }
    
    return C_OK;
}

DEFINE_THREAD_ROUTINE(score_logic, data){
    
    //NOTE: the drone and the enemy have a certain amount of "life".
    //Every time the enemy hit the drone, the drone life decrease by one
    //Every time the drone hit the enemy, the enemy life decrease by one
    //If the drone find tot hills, the drone wins
    //If the enemy "kill" the drone, the enemy wins
    //If time runs out and the drone find at least one hill, the drone wins
    
    while(game_active){
        
        //This happen if the enemy hits the drone
        vp_os_mutex_lock(&drone_score_mutex);
            if(drone_lose_score){
                if(drone_score > 0){
                    drone_score--;
                    vp_os_mutex_lock(&drone_wound_mutex);
                        drone_wounded = 1;
                    vp_os_mutex_unlock(&drone_wound_mutex);
                } else{
                    //TODO: else the drone is dead, game over!
                }
                drone_lose_score = 0;
            }
        vp_os_mutex_unlock(&drone_score_mutex);
        
        //This happen if the drone hits the enemy
        vp_os_mutex_lock(&enemy_score_mutex);
            if(enemy_lose_score){
                if(enemy_score > 0){
                    enemy_score--;
                    //TODO make the enemy aware that he's being hit
                } else {
                    //TODO: enemy is dead!!
                }
                
                enemy_lose_score = 0;
            }
        vp_os_mutex_unlock(&enemy_score_mutex);
          
        //This happen if the drone find a hill
        //when the hill points reach a certain amount, the drone win!!
        vp_os_mutex_lock(&drone_score_mutex);
            if(drone_add_score){
                drone_hill_score++;
                drone_add_score = 0;
            }
            
            if(drone_hill_score > 5){
                //TODO game over, the drone wins!!
            }
        vp_os_mutex_unlock(&drone_score_mutex);
    }
    
    return C_OK;
}


/**
 * Declare Thread / Navdata tables
 */
BEGIN_THREAD_TABLE
THREAD_TABLE_ENTRY(video_stage, 20)
THREAD_TABLE_ENTRY(video_recorder, 20)
THREAD_TABLE_ENTRY(navdata_update, 20)
THREAD_TABLE_ENTRY(ardrone_control, 20)
THREAD_TABLE_ENTRY(drone_logic, 20)
THREAD_TABLE_ENTRY(wiimote_logic, 20)
THREAD_TABLE_ENTRY(score_logic, 20)
END_THREAD_TABLE