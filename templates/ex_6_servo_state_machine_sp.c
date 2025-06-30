//DC-mode servo1_code control

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include "ecrt.h"
#include <math.h>
#include <stdbool.h>
#include <sched.h> /* sched_setscheduler() */
#include "ec_print.h"

// Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_REALTIME
#define CONFIGURE_PDOS 1

// Optional features
#define PDO_SETTING	1
#define SDO_ACCESS      1

#define MEASURE_TIMING


#define WIDTH 32
#define WINDOW 6
/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
	(B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain_r = NULL;
static ec_domain_state_t domain_r_state = {};
static ec_domain_t *domain_w = NULL;
static ec_domain_state_t domain_w_state = {};

static ec_slave_config_t *sc  = NULL;
static ec_slave_config_state_t sc_state = {};
/****************************************************************************/

// process data
static uint8_t *domain_r_pd = NULL;
static uint8_t *domain_w_pd = NULL;

#define servo1  		0,0
#define servo1_code 	0x00000083, 0x000000ae

#define servo2  		0,1
#define servo2_code 	0x00000083, 0x000000ae

#define servo3  		0,2
#define servo3_code 	0x00000083, 0x000000ae

#define servo4  		0,3
#define servo4_code 	0x00000083, 0x000000ae

#define servo5  		0,4
#define servo5_code 	0x00000083, 0x000000ae

#define servo6  		0,5
#define servo6_code 	0x00000083, 0x000000ae

#define ecc  		    0,6
#define ecc_code 	    0x00000083, 0x000000a6

int demlanlap = 0;
double t = 0.0;
double time_recorded = 0;
static uint8_t shift_index = 0;
static signed int loop_counter; 
/***************************************************************************/

//signal to turn off servo on state
static unsigned int servo_flag =0;
static unsigned int deactive;

StateCode state_code = TURN_DOWN;

static unsigned int status_word1;
static unsigned int status_word2;
static unsigned int status_word3;
static unsigned int status_word4;
static unsigned int status_word5;
static unsigned int status_word6;

static unsigned int ctrl_word1;
static unsigned int ctrl_word2;
static unsigned int ctrl_word3;
static unsigned int ctrl_word4;
static unsigned int ctrl_word5;
static unsigned int ctrl_word6;

static unsigned int tar_torq1;
static unsigned int tar_torq2;
static unsigned int tar_torq3;
static unsigned int tar_torq4;
static unsigned int tar_torq5;
static unsigned int tar_torq6;

static unsigned int tar_pos1;
static unsigned int tar_pos2;
static unsigned int tar_pos3;
static unsigned int tar_pos4;
static unsigned int tar_pos5;
static unsigned int tar_pos6;

static unsigned int mode1;
static unsigned int mode2;
static unsigned int mode3;
static unsigned int mode4;
static unsigned int mode5;
static unsigned int mode6;

static unsigned int mode_disp1;
static unsigned int mode_disp2;
static unsigned int mode_disp3;
static unsigned int mode_disp4;
static unsigned int mode_disp5;
static unsigned int mode_disp6;

static unsigned int pos_act1;
static unsigned int pos_act2;
static unsigned int pos_act3;
static unsigned int pos_act4;
static unsigned int pos_act5;
static unsigned int pos_act6;

static unsigned int tor_act1;
static unsigned int tor_act2;
static unsigned int tor_act3;
static unsigned int tor_act4;
static unsigned int tor_act5;
static unsigned int tor_act6;

static unsigned int Owrite;
static signed long Ovalue;
static unsigned int Iread;
static signed long Ivalue;

uint8_t down_sensor_pos[6] = {16, 17, 18, 19, 20, 21};
uint8_t home_sensor_pos[6] = {22, 23, 24, 25, 26, 27};
uint8_t up_sensor_pos[6] = {28, 29, 30, 31, 00, 01};

static signed long temp[8]={};
static signed long mode_disp[8]={};
static signed int pos_act[8]={};
static signed int tor_act[8]={};
static signed int pos_offset[8]={0, 0, 0, 0, 0, 0, 0, 0};

float move_value[8];
bool down_sensor[8];
bool up_sensor[8];
bool home_sensor[8];

bool servo_x_homed[6] = {0, 0, 0, 0, 0, 0};


static unsigned int counter = 0;
static unsigned int blink = 0;

float rotation1 = 0;
float rotation2 = 0;
float rotation3 = 0;    
float rotation4 = 0;
float rotation5 = 0;
float rotation6 = 0;

bool homing = true;

typedef enum
{
    TORQUE_CTRL,
    POSITION_CTRL
} OperationMode;

OperationMode Operation_Mode = TORQUE_CTRL;

//rx pdo entry 
const static ec_pdo_entry_reg_t domain_r_regs[] = 
{
        {servo1,servo1_code,0x6040,00,&ctrl_word1},
        {servo2,servo2_code,0x6040,00,&ctrl_word2},
        {servo3,servo3_code,0x6040,00,&ctrl_word3},
        {servo4,servo4_code,0x6040,00,&ctrl_word4},
        {servo5,servo5_code,0x6040,00,&ctrl_word5},
        {servo6,servo6_code,0x6040,00,&ctrl_word6},

        {servo1,servo1_code,0x6060,00,&mode1},
        {servo2,servo2_code,0x6060,00,&mode2},
        {servo3,servo3_code,0x6060,00,&mode3},
        {servo4,servo4_code,0x6060,00,&mode4},
        {servo5,servo5_code,0x6060,00,&mode5},
        {servo6,servo6_code,0x6060,00,&mode6},

        {servo1,servo1_code,0x6071,00,&tar_torq1},
        {servo2,servo2_code,0x6071,00,&tar_torq2},
        {servo3,servo3_code,0x6071,00,&tar_torq3},
        {servo4,servo4_code,0x6071,00,&tar_torq4},
        {servo5,servo5_code,0x6071,00,&tar_torq5},
        {servo6,servo6_code,0x6071,00,&tar_torq6},

        {servo1,servo1_code,0x607a,00,&tar_pos1},
        {servo2,servo2_code,0x607a,00,&tar_pos2},
        {servo3,servo3_code,0x607a,00,&tar_pos3},
        {servo4,servo4_code,0x607a,00,&tar_pos4},
        {servo5,servo5_code,0x607a,00,&tar_pos5},
        {servo6,servo6_code,0x607a,00,&tar_pos6},

        {ecc,ecc_code,0x7003,0x01,&Owrite},

        {}
};

//tx pdo entry
const static ec_pdo_entry_reg_t domain_w_regs[] = 
{
        {servo1,servo1_code,0x6041,00,&status_word1},
        {servo2,servo2_code,0x6041,00,&status_word2},
        {servo3,servo3_code,0x6041,00,&status_word3},
        {servo4,servo4_code,0x6041,00,&status_word4},
        {servo5,servo5_code,0x6041,00,&status_word5},
        {servo6,servo6_code,0x6041,00,&status_word6},

        {servo1,servo1_code,0x6061,00,&mode_disp1},
        {servo2,servo2_code,0x6061,00,&mode_disp2},
        {servo3,servo3_code,0x6061,00,&mode_disp3},
        {servo4,servo4_code,0x6061,00,&mode_disp4},
        {servo5,servo5_code,0x6061,00,&mode_disp5},
        {servo6,servo6_code,0x6061,00,&mode_disp6},

        {servo1,servo1_code,0x6064,00,&pos_act1},
        {servo2,servo2_code,0x6064,00,&pos_act2},
        {servo3,servo3_code,0x6064,00,&pos_act3},
        {servo4,servo4_code,0x6064,00,&pos_act4},
        {servo5,servo5_code,0x6064,00,&pos_act5},
        {servo6,servo6_code,0x6064,00,&pos_act6},

        {servo1,servo1_code,0x6077,00,&tor_act1},
        {servo2,servo2_code,0x6077,00,&tor_act2},
        {servo3,servo3_code,0x6077,00,&tor_act3},
        {servo4,servo4_code,0x6077,00,&tor_act4},
        {servo5,servo5_code,0x6077,00,&tor_act5},
        {servo6,servo6_code,0x6077,00,&tor_act6},

        {ecc,ecc_code,0x6003,0x01,&Iread},     
        
        {}
};





static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};


/*****************************************************************************/

#if PDO_SETTING

/* Master 0, Slave 0, "R88D-KN01H-ECT      "
 * Vendor ID:       0x00000083
 * Product code:    0x00000005
 * Revision number: 0x00020001
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x607a, 0x00, 32}, /* target position*/
    {0x60ff, 0x00, 32}, 
    {0x6071, 0x00, 16}, /* Target torque */
    {0x6060, 0x00, 8},  /* Modes of operation */
    {0x60b8, 0x00, 16}, /* Touch probe function */
    {0x607f, 0x00, 32}, 
    // {0x6098, 0x00, 8},  /* Homing */

    {0x603f, 0x00, 16}, /* Error code */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x6061, 0x00, 8},  /* Modes of operation display */
    {0x60b9, 0x00, 16}, /* Touch probe status */
    {0x60ba, 0x00, 32}, /* Touch probe pos1 pos value */
    {0x60bc, 0x00, 32}, /* Touch probe pos2 pos value */
    {0x60fd, 0x00, 32}, /* Digital inputs */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 7, slave_0_pdo_entries + 0},  /* RxPDO: Controlword + Target torque + Mode + Touch probe */
    {0x1A00, 9, slave_0_pdo_entries + 7},  /* TxPDO: các phản hồi */
};


ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},   // RxPDO 1702h
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},  // TxPDO 1B02h
    {0xff}
};


/* Master 0, Slave 6, "NX-ECC202"
 * Vendor ID:       0x00000083
 * Product code:    0x000000a6
 * Revision number: 0x00010002
 */

ec_pdo_entry_info_t slave_6_pdo_entries[] = {
    {0x7003, 0x01, 32},
    {0x3003, 0x04, 128},
    {0x3006, 0x04, 128},
    {0x2002, 0x01, 8},
    {0x0000, 0x00, 8}, /* Gap */
    {0x6003, 0x01, 32},
};

ec_pdo_info_t slave_6_pdos[] = {
    {0x1604, 1, slave_6_pdo_entries + 0},
    {0x1bf8, 2, slave_6_pdo_entries + 1},
    {0x1bff, 1, slave_6_pdo_entries + 3},
    {0x1bf4, 1, slave_6_pdo_entries + 4},
    {0x1a00, 1, slave_6_pdo_entries + 5},
};

ec_sync_info_t slave_6_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_6_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 4, slave_6_pdos + 1, EC_WD_DISABLE},
    {0xff}
};



#endif
/***********************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
	struct timespec result;

	if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) 
	{
		result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
	} 
	else 
	{
		result.tv_sec = time1.tv_sec + time2.tv_sec;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
	}

	return result;
}

/*****************************************************************************/

void endsignal(int sig)
{	
	servo_flag = 1;
	signal( SIGINT , SIG_DFL );
}

/*****************************************************************************/


void check_domain_r_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain_r, &ds);

	//struct timespec time_wc1,time_wc2;
	if (ds.working_counter != domain_r_state.working_counter)
		printf("domain_r: WC %u.\n", ds.working_counter);
	if (ds.wc_state != domain_r_state.wc_state)
        	printf("domain_r: State %u.\n", ds.wc_state);

    domain_r_state = ds;
}


void check_domain_w_state(void)
{
    ec_domain_state_t ds2;
    ecrt_domain_state(domain_w, &ds2);

	if (ds2.working_counter != domain_w_state.working_counter)
		printf("domain_w: WC %u.\n", ds2.working_counter);
	if (ds2.wc_state != domain_w_state.wc_state)
        	printf("domain_w: State %u.\n", ds2.wc_state);

    domain_w_state = ds2;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/****************************************************************************/

void cyclic_task()
{
    struct timespec wakeupTime, time;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime;
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
                latency_min_ns = 0xffffffff, latency_max_ns = 0,
                period_min_ns = 0xffffffff, period_max_ns = 0,
                exec_min_ns = 0xffffffff, exec_max_ns = 0;
#endif
    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

#ifdef MEASURE_TIMING
    lastStartTime = wakeupTime;
#endif

	while(1) 
	{

		if(deactive==1)
		{
			break;
		}

		wakeupTime = timespec_add(wakeupTime, cycletime);
     	clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);

        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns  = DIFF_NS(lastStartTime, startTime);
#endif
		
		// writter_receive(master);
 	   	ecrt_master_receive(master);
   		ecrt_domain_process(domain_r);
   		ecrt_domain_process(domain_w);


        temp[0]=EC_READ_U16(domain_w_pd + status_word1); // read 0x6041
        temp[1]=EC_READ_U16(domain_w_pd + status_word2); // read 0x6041
        temp[2]=EC_READ_U16(domain_w_pd + status_word3); // read 0x6041
        temp[3]=EC_READ_U16(domain_w_pd + status_word4); // read 0x6041
        temp[4]=EC_READ_U16(domain_w_pd + status_word5); // read 0x6041
        temp[5]=EC_READ_U16(domain_w_pd + status_word6); // read 0x6041

        mode_disp[0]=EC_READ_U8(domain_w_pd + mode_disp1); // read 0x6061
        mode_disp[1]=EC_READ_U8(domain_w_pd + mode_disp2); // read 0x6061
        mode_disp[2]=EC_READ_U8(domain_w_pd + mode_disp3); // read 0x6061
        mode_disp[3]=EC_READ_U8(domain_w_pd + mode_disp4); // read 0x6061
        mode_disp[4]=EC_READ_U8(domain_w_pd + mode_disp5); // read 0x6061
        mode_disp[5]=EC_READ_U8(domain_w_pd + mode_disp6); // read 0x6061

        pos_act[0]=EC_READ_U32(domain_w_pd + pos_act1); // read 0x6064
        pos_act[1]=EC_READ_U32(domain_w_pd + pos_act2); // read 0x6064
        pos_act[2]=EC_READ_U32(domain_w_pd + pos_act3); // read 0x6064
        pos_act[3]=EC_READ_U32(domain_w_pd + pos_act4); // read 0x6064
        pos_act[4]=EC_READ_U32(domain_w_pd + pos_act5); // read 0x6064
        pos_act[5]=EC_READ_U32(domain_w_pd + pos_act6); // read 0x6064

        tor_act[0]=(float)(EC_READ_S16(domain_w_pd + tor_act1)); // read 0x6064
        tor_act[1]=(float)(EC_READ_S16(domain_w_pd + tor_act2)); // read 0x6064
        tor_act[2]=(float)(EC_READ_S16(domain_w_pd + tor_act3)); // read 0x6064
        tor_act[3]=(float)(EC_READ_S16(domain_w_pd + tor_act4)); // read 0x6064
        tor_act[4]=(float)(EC_READ_S16(domain_w_pd + tor_act5)); // read 0x6064
        tor_act[5]=(float)(EC_READ_S16(domain_w_pd + tor_act6)); // read 0x6064



        rotation1 = (float)(pos_act[0] + pos_offset[0]) / 1048576.0f;     // to rotation
        rotation2 = (float)(pos_act[1] + pos_offset[1]) / 1048576.0f;
        rotation3 = (float)(pos_act[2] + pos_offset[2]) / 1048576.0f;
        rotation4 = (float)(pos_act[3] + pos_offset[3]) / 1048576.0f;
        rotation5 = (float)(pos_act[4] + pos_offset[4]) / 1048576.0f;
        rotation6 = (float)(pos_act[5] + pos_offset[5]) / 1048576.0f;
        

        if (counter) 
		{
            counter--;
        } 	
		else 
		{ // do this at 1 Hz
            counter = 100;
			check_master_state();
#ifdef MEASURE_TIMING
            printf("period     %10u ... %10u\n", period_min_ns, period_max_ns);
            printf("exec       %10u ... %10u\n", exec_min_ns, exec_max_ns);
            printf("latency    %10u ... %10u\n", latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
            printf("============================\n");
            printf("Time = %f\n", t);
            ec_print_header();
            ec_print_row_hex("Status (0x6041)",
                temp[0], temp[1], temp[2],
                temp[3], temp[4], temp[5]);
            ec_print_row_hex("Mode (0x6061)",
                mode_disp[0], mode_disp[1], mode_disp[2],
                mode_disp[3], mode_disp[4], mode_disp[5]);
            ec_print_row_float("Rotation (rounds)",
                rotation1, rotation2, rotation3,
                rotation4, rotation5, rotation6);
            ec_print_row_float("Command values",
                move_value[0], move_value[1], move_value[2],
                move_value[3], move_value[4], move_value[5]);
            ec_print_row_float("Actual values",
                tor_act[0], tor_act[1], tor_act[2],
                tor_act[3], tor_act[4], tor_act[5]);
            ec_print_row_bool("Up sensor",
                up_sensor[0], up_sensor[1], up_sensor[2],
                up_sensor[3], up_sensor[4], up_sensor[5]);
            ec_print_row_bool("Home sensor",
                home_sensor[0], home_sensor[1], home_sensor[2],
                home_sensor[3], home_sensor[4], home_sensor[5]);
            ec_print_row_bool("Down sensor",
                down_sensor[0], down_sensor[1], down_sensor[2],
                down_sensor[3], down_sensor[4], down_sensor[5]);
            ec_print_line();
            ec_print_uint32_binary(Ivalue);
            ec_print_state_code(state_code);
#endif
            blink = !blink;
        }


        


        if ( (temp[0] & 0b00001000) == 0b00001000 )
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word1, 0b10000000);
        }
        else if ((temp[0] & 0x004f) == 0x0040)
        {
           EC_WRITE_U16(domain_r_pd+ctrl_word1, 0x0006);
        }
        
        if ( (temp[1] & 0b00001000) == 0b00001000 )
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word2, 0b10000000);
        }
        else if ((temp[1] & 0x004f) == 0x0040)
        {
           EC_WRITE_U16(domain_r_pd+ctrl_word2, 0x0006);
        }
        
        if ( (temp[2] & 0b00001000) == 0b00001000 )
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word3, 0b10000000);
        }
        else if ((temp[2] & 0x004f) == 0x0040)
        {
           EC_WRITE_U16(domain_r_pd+ctrl_word3, 0x0006);
        }
        
        if ( (temp[3] & 0b00001000) == 0b00001000 )
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word4, 0b10000000);
        }
        else if ((temp[3] & 0x004f) == 0x0040)
        {
           EC_WRITE_U16(domain_r_pd+ctrl_word4, 0x0006);
        }
        
        if ( (temp[4] & 0b00001000) == 0b00001000 )
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word5, 0b10000000);
        }
        else if ((temp[4] & 0x004f) == 0x0040)
        {
           EC_WRITE_U16(domain_r_pd+ctrl_word5, 0x0006);
        }
        
        if ( (temp[5] & 0b00001000) == 0b00001000 )
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word6, 0b10000000);
        }
        else if ((temp[5] & 0x004f) == 0x0040)
        {
           EC_WRITE_U16(domain_r_pd+ctrl_word6, 0x0006);
        }



    SERVO_START_LABEL:

        if (((temp[0] & 0x006f) == 0x0021) & ((temp[1] & 0x006f) == 0x0021) & ((temp[2] & 0x006f) == 0x0021)
            & ((temp[3] & 0x006f) == 0x0021) & ((temp[4] & 0x006f) == 0x0021) & ((temp[5] & 0x006f) == 0x0021))
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word1, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word2, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word3, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word4, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word5, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word6, 0x0007);

            if (Operation_Mode == TORQUE_CTRL)
            {
                EC_WRITE_U8(domain_r_pd+mode1, 10);
                EC_WRITE_U8(domain_r_pd+mode2, 10);
                EC_WRITE_U8(domain_r_pd+mode3, 10);
                EC_WRITE_U8(domain_r_pd+mode4, 10);
                EC_WRITE_U8(domain_r_pd+mode5, 10);
                EC_WRITE_U8(domain_r_pd+mode6, 10);
            }
            else if (Operation_Mode == POSITION_CTRL)
            {
                EC_WRITE_U8(domain_r_pd+mode1, 8);
                EC_WRITE_U8(domain_r_pd+mode2, 8);
                EC_WRITE_U8(domain_r_pd+mode3, 8);
                EC_WRITE_U8(domain_r_pd+mode4, 8);
                EC_WRITE_U8(domain_r_pd+mode5, 8);
                EC_WRITE_U8(domain_r_pd+mode6, 8);
            }
            
            // time_recorded = t;

        }
        else if(((temp[0]&0x006f) == 0x0023) & ((temp[1]&0x006f) == 0x0023) & ((temp[2]&0x006f) == 0x0023)
            & ((temp[3]&0x006f) == 0x0023) & ((temp[4]&0x006f) == 0x0023) & ((temp[5]&0x006f) == 0x0023))
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word1, 0x000f);       /* Turn on all servo*/
            EC_WRITE_U16(domain_r_pd+ctrl_word2, 0x000f);
            EC_WRITE_U16(domain_r_pd+ctrl_word3, 0x000f);
            EC_WRITE_U16(domain_r_pd+ctrl_word4, 0x000f);
            EC_WRITE_U16(domain_r_pd+ctrl_word5, 0x000f);
            EC_WRITE_U16(domain_r_pd+ctrl_word6, 0x000f);
        }
        else if(((temp[0]&0x006f) == 0x0027) & ((temp[1]&0x006f) == 0x0027) & ((temp[2]&0x006f) == 0x0027)
            & ((temp[3]&0x006f) == 0x0027) & ((temp[4]&0x006f) == 0x0027) & ((temp[5]&0x006f) == 0x0027))        // this servo is on
        {
            if (Operation_Mode == TORQUE_CTRL)
            {
                EC_WRITE_U8(domain_r_pd+mode1, 10);
                EC_WRITE_U8(domain_r_pd+mode2, 10);
                EC_WRITE_U8(domain_r_pd+mode3, 10);
                EC_WRITE_U8(domain_r_pd+mode4, 10);
                EC_WRITE_U8(domain_r_pd+mode5, 10);
                EC_WRITE_U8(domain_r_pd+mode6, 10);
            }
            else if (Operation_Mode == POSITION_CTRL)
            {
                EC_WRITE_U8(domain_r_pd+mode1, 8);
                EC_WRITE_U8(domain_r_pd+mode2, 8);
                EC_WRITE_U8(domain_r_pd+mode3, 8);
                EC_WRITE_U8(domain_r_pd+mode4, 8);
                EC_WRITE_U8(domain_r_pd+mode5, 8);
                EC_WRITE_U8(domain_r_pd+mode6, 8);
            }
            
            EC_WRITE_U16(domain_r_pd+ctrl_word1, 0x001f);       /* Turn on all servo*/
            EC_WRITE_U16(domain_r_pd+ctrl_word2, 0x001f);
            EC_WRITE_U16(domain_r_pd+ctrl_word3, 0x001f);
            EC_WRITE_U16(domain_r_pd+ctrl_word4, 0x001f);
            EC_WRITE_U16(domain_r_pd+ctrl_word5, 0x001f);
            EC_WRITE_U16(domain_r_pd+ctrl_word6, 0x001f);


            // Begin of Controller
            t += 0.001; // increment time for simulation purposes


            if (loop_counter)
            {
                loop_counter--;
            }
            else
            {
                loop_counter = 30;
                uint32_t pattern = ((1U << WINDOW) - 1) << (WIDTH - WINDOW - shift_index);

                // Write the pattern to the process data
                EC_WRITE_U32(domain_r_pd + Owrite, pattern);

                // Update the shift index
                shift_index++;
                if (shift_index > WIDTH - WINDOW) 
                {
                    shift_index = 0;  // Reset to loop back
                }
            }


            Ivalue = EC_READ_U32(domain_w_pd + Iread); // read the value from the process data
            // EC_WRITE_U32(domain_r_pd + Owrite, 0xf0f0f0f0); // read the value from the process data
            
            for (int i = 0; i < 6; i ++)
            {
                uint32_t down_check = Ivalue >> down_sensor_pos[i];
                uint32_t home_check = Ivalue >> home_sensor_pos[i];
                uint32_t up_check = Ivalue >> up_sensor_pos[i];
                down_sensor[i] = down_check & 0x1;
                home_sensor[i] = home_check & 0x1;
                up_sensor[i] = up_check & 0x1;
            }
           


            // State machine for homing setup and operation preparation
            switch (state_code)
            {
            case TURN_DOWN:
                for (int i = 0; i < 6; i++)
                {
                    if (down_sensor[i] == 0x00000001)
                    {
                        pos_offset[i] = -pos_act[i];
                        move_value[i] = 0;
                    }
                    else
                    {
                        // move_value[i] =   200 * sin(2 * 3.14159 * 0.5 * t); // simulate a sine wave position
                        move_value[0] = -3000;
                        move_value[1] = -3000;
                        move_value[2] = -3000;
                        move_value[3] = 3000;
                        move_value[4] = -3000;
                        move_value[5] = -3000;
                    }
                    
                }
                if (down_sensor[0] & down_sensor[1] & down_sensor[2] & down_sensor[3] & down_sensor[4] & down_sensor[5])
                {
                    state_code = GO_HOME;
                }
                
                break;
            

            case GO_HOME:
                for (int i = 0; i < 6; i++)
                {
                    if (home_sensor[i] == 0x00000001)
                    {
                        
                        move_value[i] = 0;
                    }
                    else
                    {
                        move_value[i] =   200 * sin(2 * 3.14159 * 0.5 * t); // simulate a sine wave position
                    }
                }

                if (home_sensor[0] & home_sensor[1] & home_sensor[2] & home_sensor[3] & home_sensor[4] & home_sensor[5])
                {
                    if (t - time_recorded > 2)
                    {
                        // need a wait here?
                        EC_WRITE_U16(domain_r_pd+ctrl_word1, 0x0000);
                        EC_WRITE_U16(domain_r_pd+ctrl_word2, 0x0000);
                        EC_WRITE_U16(domain_r_pd+ctrl_word3, 0x0000);
                        EC_WRITE_U16(domain_r_pd+ctrl_word4, 0x0000);
                        EC_WRITE_U16(domain_r_pd+ctrl_word5, 0x0000);
                        EC_WRITE_U16(domain_r_pd+ctrl_word6, 0x0000);

                        for (int i = 0; i < 6; i ++)
                            pos_offset[i] = -pos_act[i];
                            
                        state_code = OPERATE;
                    }

                }
                else
                {

                    time_recorded = t;
                }

                
                break;

            case OPERATE:
                if (home_sensor[0] & home_sensor[1] & home_sensor[2] & home_sensor[3] & home_sensor[4] & home_sensor[5])
                {
                    for (int i = 0; i < 6; i ++)
                        pos_offset[i] = -pos_act[i];

                }

                Operation_Mode = POSITION_CTRL;

                if (Operation_Mode == TORQUE_CTRL)
                {
                    EC_WRITE_U8(domain_r_pd+mode1, 10);
                    EC_WRITE_U8(domain_r_pd+mode2, 10);
                    EC_WRITE_U8(domain_r_pd+mode3, 10);
                    EC_WRITE_U8(domain_r_pd+mode4, 10);
                    EC_WRITE_U8(domain_r_pd+mode5, 10);
                    EC_WRITE_U8(domain_r_pd+mode6, 10);
                }
                if (Operation_Mode == POSITION_CTRL)
                {
                    EC_WRITE_U8(domain_r_pd+mode1, 8);
                    EC_WRITE_U8(domain_r_pd+mode2, 8);
                    EC_WRITE_U8(domain_r_pd+mode3, 8);
                    EC_WRITE_U8(domain_r_pd+mode4, 8);
                    EC_WRITE_U8(domain_r_pd+mode5, 8);
                    EC_WRITE_U8(domain_r_pd+mode6, 8);
                }
                // did shutdown is need for change mode of operation?

                
                move_value[0] = 300000 * sin(2 * 3.14159 * 0.5 * t); // simulate a sine wave position

                break;


            default:
                break;
            }



            if (Operation_Mode == TORQUE_CTRL)
            {
                EC_WRITE_S16(domain_r_pd+tar_torq1, move_value[0]); // set target position
                EC_WRITE_S16(domain_r_pd+tar_torq2, move_value[1]); // set target position
                EC_WRITE_S16(domain_r_pd+tar_torq3, move_value[2]); // set target position
                EC_WRITE_S16(domain_r_pd+tar_torq4, move_value[3]); // set target position
                EC_WRITE_S16(domain_r_pd+tar_torq5, move_value[4]); // set target position
                EC_WRITE_S16(domain_r_pd+tar_torq6, move_value[5]); // set target position
            }
            
            else if (Operation_Mode == POSITION_CTRL)
            {
                EC_WRITE_S32(domain_r_pd+tar_pos1,move_value[0]);
                EC_WRITE_S32(domain_r_pd+tar_pos2,move_value[0]);
                EC_WRITE_S32(domain_r_pd+tar_pos3,move_value[0]);
                EC_WRITE_S32(domain_r_pd+tar_pos4,move_value[0]);
                EC_WRITE_S32(domain_r_pd+tar_pos5,move_value[0]);
                EC_WRITE_S32(domain_r_pd+tar_pos6,0);
            }
            

            // End off Controller

            // EC_WRITE_U16(domain_r_pd+ctrl_word1, 0x001f);
            // EC_WRITE_U16(domain_r_pd+ctrl_word2, 0x001f);
            // EC_WRITE_U16(domain_r_pd+ctrl_word3, 0x001f);
            // EC_WRITE_U16(domain_r_pd+ctrl_word4, 0x001f);
            // EC_WRITE_U16(domain_r_pd+ctrl_word5, 0x001f);
            // EC_WRITE_U16(domain_r_pd+ctrl_word6, 0x001f);

        }
        // printf("Mode: %d\n", disp_mode)



		clock_gettime(CLOCK_TO_USE, &time);
		ecrt_master_application_time(master, TIMESPEC2NS(time));

		if (sync_ref_counter) 
		{
			sync_ref_counter--;
		} 
		else 
		{
			sync_ref_counter = 1; // sync every cycle
			ecrt_master_sync_reference_clock(master);
		}

		ecrt_master_sync_slave_clocks(master);

		
		
		// send process data
		ecrt_domain_queue(domain_r);
		ecrt_domain_queue(domain_w);
		
		ecrt_master_send(master);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
        exec_ns = DIFF_NS(startTime, endTime);

        if (latency_ns > latency_max_ns) latency_max_ns = latency_ns;
        if (latency_ns < latency_min_ns) latency_min_ns = latency_ns;
        if (period_ns  > period_max_ns)  period_max_ns  = period_ns;
        if (period_ns  < period_min_ns)  period_min_ns  = period_ns;
        if (exec_ns    > exec_max_ns)    exec_max_ns    = exec_ns;
        if (exec_ns    < exec_min_ns)    exec_min_ns    = exec_ns;

        lastStartTime = startTime;
#endif
	}
}

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
	
	
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) 
    {
		perror("mlockall failed");
		return -1;
    }

    master = ecrt_request_master(0);

    if (!master)
	        return -1;

    domain_r = ecrt_master_create_domain(master);
    
    if (!domain_r)
	        return -1;
	
    domain_w = ecrt_master_create_domain(master);

    if (!domain_w)
	        return -1;
			
	 	
	    
     
    

#if SDO_ACCESS

    // setup servo 1
    // ==========================================================================
    if (!(sc = ecrt_master_slave_config(master, servo1, servo1_code))) 
    {
	fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
    }  
    
    // if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    // {
    //     return -1;
    // }

    printf("Configuring PDOs 1...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure PDOs 1.\n");
        return -1;
    }

    ecrt_slave_config_dc(sc,
        0x0300,
        1000000,4400000,0,0);  


    // setup servo 2
    // ==========================================================================
    if (!(sc = ecrt_master_slave_config(master, servo2, servo2_code))) 
    {
	fprintf(stderr, "Failed to get slave 2 configuration.\n");
        return -1;
    }  
    
    // if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    // {
    //     return -1;
    // }

    printf("Configuring PDOs 2...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure PDOs 2.\n");
        return -1;
    }

    ecrt_slave_config_dc(sc,
        0x0300,
        1000000,4400000,0,0);  


    
    // setup servo 3
    // ==========================================================================
    if (!(sc = ecrt_master_slave_config(master, servo3, servo3_code))) 
    {
	fprintf(stderr, "Failed to get slave 3 configuration.\n");
        return -1;
    }  
    
    // if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    // {
    //     return -1;
    // }


    printf("Configuring PDOs 3...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure PDOs 3.\n");
        return -1;
    }

    ecrt_slave_config_dc(sc,
        0x0300,
        1000000,4400000,0,0);  

    // setup servo 4
    // ==========================================================================
    if (!(sc = ecrt_master_slave_config(master, servo4, servo4_code))) 
    {
	fprintf(stderr, "Failed to get slave 4 configuration.\n");
        return -1;
    }  
    
    // if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    // {
    //     return -1;
    // }

    printf("Configuring PDOs 4...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure PDOs 4.\n");
        return -1;
    }


    ecrt_slave_config_dc(sc,
        0x0300,
        1000000,4400000,0,0);  



    // setup servo 5
    // ==========================================================================
    if (!(sc = ecrt_master_slave_config(master, servo5, servo5_code))) 
    {
	fprintf(stderr, "Failed to get slave 5 configuration.\n");
        return -1;
    }  
    
    // if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    // {
    //     return -1;
    // }

    printf("Configuring PDOs 5...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure PDOs 5.\n");
        return -1;
    }

    ecrt_slave_config_dc(sc,
        0x0300,
        1000000,4400000,0,0);  

    // setup servo 6
    // ==========================================================================
    if (!(sc = ecrt_master_slave_config(master, servo6, servo6_code))) 
    {
	fprintf(stderr, "Failed to get slave 6 configuration.\n");
        return -1;
    }  
    
    // if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    // {
    //     return -1;
    // }

    if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    {
        return -1;
    }

    printf("Configuring PDOs 6...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure PDOs 6.\n");
        return -1;
    }


    ecrt_slave_config_dc(sc,
        0x0300,
        1000000,4400000,0,0);  




    // setup IO
    // ==========================================================================
    if (!(sc = ecrt_master_slave_config(master, ecc, ecc_code))) 
    {
	fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
    }  

    printf("Configuring PDOs 1...\n");
	
    if (ecrt_slave_config_pdos(sc, EC_END, slave_6_syncs)) 
    {
        fprintf(stderr, "Failed to configure PDOs 1.\n");
        return -1;
    }

    ecrt_slave_config_dc(sc,
        0x0300,
        1000000,4400000,0,0);  


#endif


	if (ecrt_domain_reg_pdo_entry_list(domain_r, domain_r_regs))
        {
        	fprintf(stderr, "1st motor RX_PDO entry registration failed!\n");
        	return -1;
    	}	
	
	if (ecrt_domain_reg_pdo_entry_list(domain_w, domain_w_regs)) 
	{
        	fprintf(stderr, "1st motor TX_PDO entry registration failed!\n");
        	return -1;
    	}
		
	

    printf("Activating master...\n");
	
    
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain_r_pd = ecrt_domain_data(domain_r))) 
    {
        return -1;
    }

    if (!(domain_w_pd = ecrt_domain_data(domain_w))) 
    {
        return -1;
    }


    
    
    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.\n", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    
    pid_t pid = getpid();

    if (setpriority(PRIO_PROCESS, pid, -20))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));

	signal( SIGINT , endsignal );		
	printf("Starting cyclic function.\n");
    cyclic_task();

	ecrt_release_master(master);
	
    return 0;	
}