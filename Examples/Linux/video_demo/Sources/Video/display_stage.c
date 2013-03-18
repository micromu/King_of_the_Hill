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
#include "detection_functions.c"


// Funcs pointer definition
const vp_api_stage_funcs_t display_stage_funcs = {
    NULL,
    (vp_api_stage_open_t) display_stage_open,
    (vp_api_stage_transform_t) display_stage_transform,
    (vp_api_stage_close_t) display_stage_close
};

C_RESULT display_stage_open(display_stage_cfg_t *cfg){
    return C_OK;
}

C_RESULT display_stage_transform(display_stage_cfg_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out){
    
    //NOTE: this is here because is easier to pick up the frame.
    // I probably should move this in a thread of is own
    show_gui((uint8_t*)in->buffers[in->indexBuffer]);

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
