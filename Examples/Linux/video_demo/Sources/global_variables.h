//CONSTANTS
//TODO: find the right values
#define HILL_MIN_DISTANCE 200
#define HILL_MAX_DISTANCE 500
#define ERROR_FROM_CENTER_FOR_HILL 5

//NOTE: nintendo advice a distance between the wiimote and the sensor bar of 3/10m
#define ENEMY_MIN_DISTANCE 200
#define ENEMY_MAX_DISTANCE 500
#define ENEMY_SHOOTING_DISTANCE 300
#define ERROR_FROM_CENTER_FOR_ENEMY 5

#define magazine_capacity 5

#define YAW_COEFF 0.007
#define THETA_COEFF 0.007

int debugging = 0;

//DRONE LOGIC
//These flags are set by the gtk thread
int game_active = 1;
int match_active = 0;
int takeoff = 0;
//These flags are set in the video stream analisys
int hill_in_sight = 0;
int enemy_in_sight = 0;
int hill_distance = 0; //this is expressed in cm
int enemy_distance = 0; //this is expressed in cm
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
int drone_score = 10;
int drone_hill_score = 0;
int enemy_score = 10;

vp_os_mutex_t drone_score_mutex = PTHREAD_MUTEX_INITIALIZER;
vp_os_mutex_t enemy_score_mutex = PTHREAD_MUTEX_INITIALIZER;
int drone_add_score = 0;
int enemy_add_score = 0;
int enemy_lose_score = 0;
int drone_lose_score = 0;

