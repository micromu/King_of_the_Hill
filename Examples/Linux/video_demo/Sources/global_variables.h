//CONSTANTS
//TODO: find the right values
#define hill_max_distance 50
#define hill_min_distance 10

#define enemy_max_distance 50
#define enemy_min_distance 20
#define enemy_shooting_distance 30

#define magazine_capacity 5

//DRONE LOGIC
//These flags are set by the gtk thread
int game_active = 1;
int match = 0;
//These flags are set in the video stream analisys
int hill_in_sight = 0;
int enemy_in_sight = 0;
int hill_distance = 0;
int enemy_distance = 0;
int hill_offset_from_center = 0;
int enemy_offset_from_center = 0;
int drone_above_hill = 0; //the drone is above the target hill
int enemy_on_target = 0;//the enemy is in the center of the image

//These flags are set by the score logic thread
/*vp_os_mutex_init(vp_os_mutex_t *mutex);
vp_os_mutex_destroy(vp_os_mutex_t *mutex);
vp_os_mutex_lock(vp_os_mutex_t *mutex);
vp_os_mutex_unlock(vp_os_mutex_t *mutex);*/

vp_os_mutex_t drone_wound_mutex = PTHREAD_MUTEX_INITIALIZER;
vp_os_mutex_t enemy_wound_mutex = PTHREAD_MUTEX_INITIALIZER;
int drone_wounded = 0;
int enemy_wounded = 0;

//VIDEO STREAM
int active_cam = 0; //who set this?


//SCORE LOGIC
//remember to set to 0/false drone_wounded and enemy_wounded after adding to te total score
// e se potesse fare punto solo il drone, e il nemico si limitasse a stordirlo per tot secondi?
int drone_score = 10;
int enemy_score = 10;

vp_os_mutex_t drone_score_mutex = PTHREAD_MUTEX_INITIALIZER;
vp_os_mutex_t enemy_score_mutex = PTHREAD_MUTEX_INITIALIZER;
int drone_add_score = 0;
int enemy_add_score = 0;

