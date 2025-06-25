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
#define servo1_code 	0x00000083, 0x00000007

#define servo2  		0,1
#define servo2_code 	0x00000083, 0x00000007

#define servo3  		0,2
#define servo3_code 	0x00000083, 0x00000005

#define servo4  		0,3
#define servo4_code 	0x00000083, 0x00000005

#define servo5  		0,4
#define servo5_code 	0x00000083, 0x00000005

#define servo6  		0,5
#define servo6_code 	0x00000083, 0x00000005

int demlanlap = 0;
double t = 0;
/***************************************************************************/

//signal to turn off servo on state
static unsigned int servo_flag =0;
static unsigned int deactive;

static unsigned int mode_display;
static unsigned int pos_act;

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

static unsigned int mode1;
static unsigned int mode2;
static unsigned int mode3;
static unsigned int mode4;
static unsigned int mode5;
static unsigned int mode6;



static signed long temp[8]={};

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

        {servo1,servo1_code,0x6061,00,&mode_display},

        
        {}
};




float move_value = 0;
static unsigned int counter = 0;
static unsigned int blink = 0;
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
    {0x607a, 0x00, 32}, 
    {0x60ff, 0x00, 32}, 
    {0x6071, 0x00, 16}, /* Target torque */
    {0x6060, 0x00, 8},  /* Modes of operation */
    {0x60b8, 0x00, 16}, /* Touch probe function */
    {0x607f, 0x00, 32}, 

    {0x603f, 0x00, 16}, /* Error code */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, 
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

        

        if (counter) 
		{
            counter--;
        } 	
		else 
		{ // do this at 1 Hz
            counter = FREQUENCY;
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
            ec_print_header();
            ec_print_row_hex("Status",
                temp[0], temp[1], temp[2],
                temp[3], temp[4], temp[5]);
            ec_print_line();
#endif
            blink = !blink;
        }


        temp[0]=EC_READ_U16(domain_w_pd + status_word1); // read 0x6041
        temp[1]=EC_READ_U16(domain_w_pd + status_word2); // read 0x6041
        temp[2]=EC_READ_U16(domain_w_pd + status_word3); // read 0x6041
        temp[3]=EC_READ_U16(domain_w_pd + status_word4); // read 0x6041
        temp[4]=EC_READ_U16(domain_w_pd + status_word5); // read 0x6041
        temp[5]=EC_READ_U16(domain_w_pd + status_word6); // read 0x6041


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

        if (((temp[0] & 0x006f) == 0x0021) & ((temp[1] & 0x006f) == 0x0021) & ((temp[2] & 0x006f) == 0x0021)
            & ((temp[3] & 0x006f) == 0x0021) & ((temp[4] & 0x006f) == 0x0021) & ((temp[5] & 0x006f) == 0x0021))
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word1, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word2, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word3, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word4, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word5, 0x0007);
            EC_WRITE_U16(domain_r_pd+ctrl_word6, 0x0007);

            EC_WRITE_U8(domain_r_pd+mode1, 10);
            EC_WRITE_U8(domain_r_pd+mode2, 10);
            EC_WRITE_U8(domain_r_pd+mode3, 10);
            EC_WRITE_U8(domain_r_pd+mode4, 10);
            EC_WRITE_U8(domain_r_pd+mode5, 10);
            EC_WRITE_U8(domain_r_pd+mode6, 10);
        }
        else if(((temp[0]&0x006f) == 0x0023) & ((temp[1]&0x006f) == 0x0023) & ((temp[2]&0x006f) == 0x0023)
            & ((temp[3]&0x006f) == 0x0023) & ((temp[4]&0x006f) == 0x0023) & ((temp[5]&0x006f) == 0x0023))
        {
            EC_WRITE_U16(domain_r_pd+ctrl_word1, 0x000f);
            EC_WRITE_U16(domain_r_pd+ctrl_word2, 0x000f);
            EC_WRITE_U16(domain_r_pd+ctrl_word3, 0x000f);
            EC_WRITE_U16(domain_r_pd+ctrl_word4, 0x000f);
            EC_WRITE_U16(domain_r_pd+ctrl_word5, 0x000f);
            EC_WRITE_U16(domain_r_pd+ctrl_word6, 0x000f);
        }
        else if( (temp[0]&0x006f) == 0x0027)        // this servo is on
        {
            EC_WRITE_U8(domain_r_pd+mode1, 10);
            EC_WRITE_U8(domain_r_pd+mode2, 10);
            EC_WRITE_U8(domain_r_pd+mode3, 10);
            EC_WRITE_U8(domain_r_pd+mode4, 10);
            EC_WRITE_U8(domain_r_pd+mode5, 10);
            EC_WRITE_U8(domain_r_pd+mode6, 10);




            // Begin of Controller
            t += 0.001; // increment time for simulation purposes
            move_value =   50 * sin(2 * 3.14159 * 0.5 * t); // simulate a sine wave position
            // move
            
            EC_WRITE_S16(domain_r_pd+tar_torq1, move_value); // set target position
            EC_WRITE_S16(domain_r_pd+tar_torq2, move_value); // set target position
            EC_WRITE_S16(domain_r_pd+tar_torq3, move_value); // set target position
            EC_WRITE_S16(domain_r_pd+tar_torq4, move_value); // set target position
            EC_WRITE_S16(domain_r_pd+tar_torq5, move_value); // set target position
            EC_WRITE_S16(domain_r_pd+tar_torq6, move_value); // set target position



            // End off Controller

            EC_WRITE_U16(domain_r_pd+ctrl_word1, 0x001f);
            EC_WRITE_U16(domain_r_pd+ctrl_word2, 0x001f);
            EC_WRITE_U16(domain_r_pd+ctrl_word3, 0x001f);
            EC_WRITE_U16(domain_r_pd+ctrl_word4, 0x001f);
            EC_WRITE_U16(domain_r_pd+ctrl_word5, 0x001f);
            EC_WRITE_U16(domain_r_pd+ctrl_word6, 0x001f);

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
    
    if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
    {
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    {
        return -1;
    }

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
    
    if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
{
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    {
        return -1;
    }

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
    
    if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
{
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    {
        return -1;
    }


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
    
    if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
{
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    {
        return -1;
    }

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
    
    if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
{
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x607f, 0, 1000000000))     // setting max profile velocity
    {
        return -1;
    }

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
    
    if (ecrt_slave_config_sdo16(sc, 0x1C12, 01, 0x1600))        // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x1C13, 01, 0x1A00))        // PDO mapping 0x1A00 by 0x1C13
{
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc, 0x3317, 00, 0))             // PDO mapping 0x1600 by 0x1C12
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))       // speed limit selection
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc, 0x6065, 0, 0xFFFFFFFF))     // setting following error window
    {
        return -1;
    }

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