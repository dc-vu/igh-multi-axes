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


// Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_REALTIME
#define CONFIGURE_PDOS 0

// // Optional features
#define PDO_SETTING	    0
#define SDO_ACCESS      1

// #define MEASURE_TIMING

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
#define servo1_code 	0x5a65726f, 0x00029252

int demlanlap = 0;
double t = 0;
/***************************************************************************/

//signal to turn off servo on state
static unsigned int servo_flag =0;
static unsigned int deactive;
// offsets for PDO entries
static unsigned int ctrl_word   ;
static unsigned int mode  ;
// static unsigned int tar_torq    ;
// static unsigned int max_torq    ;
static unsigned int tar_pos    ;
// static unsigned int max_speed  ;
// static unsigned int touch_probe_func ;
// static unsigned int tar_vel ;
// static unsigned int error_code  ;
static unsigned int status_word;
// static unsigned int mode_display ;
static unsigned int pos_act;
// static unsigned int vel_act;
// static unsigned int torq_act;
// static unsigned int touch_probe_status;
// static unsigned int touch_probe_pos;
// static unsigned int digital_input;

int temp[8]={};
uint32_t actual_pos = 0;
uint32_t offsets = 0;
uint8_t read_offset = 1;

//rx pdo entry 
const static ec_pdo_entry_reg_t domain_r_regs[] = 
{
        {servo1,servo1_code,0x6040,00,&ctrl_word              },

        // {servo1,servo1_code,0x6060,00,&mode                   },
        // {servo1,servo1_code,0x6071,00,&tar_torq               },
        // {servo1,servo1_code,0x6072,00,&max_torq               },

        {servo1,servo1_code,0x607a,00,&tar_pos                },
        
        // {servo1,servo1_code,0x6080,00,&max_speed              },
        // {servo1,servo1_code,0x60b8,00,&touch_probe_func       },
        // {servo1,servo1_code,0x60ff,00,&tar_vel                },
        {}
};

//tx pdo entry
const static ec_pdo_entry_reg_t domain_w_regs[] = 
{
        // {servo1,servo1_code,0x603f,00,&error_code             },
        {servo1,servo1_code,0x6041,00,&status_word            },
        // {servo1,servo1_code,0x6061,00,&mode_display           },
        {servo1,servo1_code,0x6064,00,&pos_act                },
        // {servo1,servo1_code,0x606c,00,&vel_act                },
        // {servo1,servo1_code,0x6077,00,&torq_act               },
        // {servo1,servo1_code,0x60b9,00,&touch_probe_status     },
        // {servo1,servo1_code,0x60ba,00,&touch_probe_pos        },
        // {servo1,servo1_code,0x60fd,00,&digital_input          },
        {}
};




uint32_t move_value = 0;
static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};


/*****************************************************************************/

// #if PDO_SETTING


//  ec_pdo_entry_info_t slave_0_pdo_entries[] = {
//     {0x607a, 0x00, 32}, /* Target position */
//     {0x60ff, 0x00, 32}, /* Target position */
//     {0x6071, 0x00, 16}, /* Target position */
//     {0x6072, 0x00, 16}, /* Target position */
//     {0x6071, 0x00, 16}, /* Target position */

//     // {0x6040, 0x00, 16}, /* Controlword */
    
//     // {0x60b8, 0x00, 16}, /* Touch probe function */
//     // {0x60fe, 0x01, 32}, /* Physical outputs */
//     // {0x603f, 0x00, 16}, /* Error code */
//     // {0x6041, 0x00, 16}, /* Statusword */
//     // {0x6064, 0x00, 32}, /* Position actual value */
//     // {0x6077, 0x00, 16}, /* Torque actual value */
//     // {0x60f4, 0x00, 32}, /* Following error actual value */
//     // {0x60b9, 0x00, 16}, /* Touch probe status */
//     // {0x60ba, 0x00, 32}, /* Touch probe pos1 pos value */
//     // {0x60bc, 0x00, 32}, /* Touch probe pos2 pos value */
//     // {0x60fd, 0x00, 32}, /* Digital inputs */
// };

// ec_pdo_info_t slave_0_pdos[] = {
//     {0x1600, 4, slave_0_pdo_entries + 0}, /* 258th receive PDO Mapping */
//     {0x1a00, 9, slave_0_pdo_entries + 4}, /* 258th transmit PDO Mapping */
// };

// ec_sync_info_t slave_0_syncs[] = {
//     {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
//     {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
//     {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
//     {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
//     {0xff}
// };
// #endif
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

        temp[0]=EC_READ_U16(domain_w_pd + status_word);

        actual_pos = EC_READ_S32(domain_w_pd + pos_act);

        if ((read_offset == 1) || (offsets == 0))
        {
            offsets = actual_pos;
            read_offset = 0;
        }
        // temp[1]=EC_READ_S32(domain_w_pd + mode_display);

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
#endif
            blink = !blink;
        }


		// write process data

        if ((((temp[0] & 0b00001000) == 0b00001000)))
        {
            if  (t < 1)
            {
                EC_WRITE_U16(domain_r_pd+ctrl_word, 0b10000000);
            }
            
            // printf("Error");
            // printf("     0x%04X  |\n", temp[0]);
            // EC_WRITE_U8(domain_r_pd+mode, 0);
        }
        else if(servo_flag==1)
		{
			//servo off
            EC_WRITE_U16(domain_r_pd+ctrl_word, 0x0006);
            // printf("abc\n");
            // printf("     0x%04X  |\n", temp[0]);
        }
        else if( (temp[0]&0x004f) == 0x0040)
		{
            EC_WRITE_U16(domain_r_pd+ctrl_word, 0x0006);
            // EC_WRITE_U8(domain_r_pd+mode, 0);
            printf("Servo state 1\n");
            printf("     0x%04X  |\n", temp[0]);
        }

        else if( (temp[0]&0x006f) == 0x0021)
		{
            EC_WRITE_U16(domain_r_pd+ctrl_word, 0x0007);
            // EC_WRITE_U8(domain_r_pd+mode, 8);
            printf("Servo state 2\n");
            printf("     0x%04X  |\n", temp[0]);
        }
        
		else if( (temp[0]&0x006f) == 0x0023)
		{
            EC_WRITE_S32(domain_r_pd+tar_pos, offsets);
            EC_WRITE_U16(domain_r_pd+ctrl_word, 0x000f);
            // EC_WRITE_U8(domain_r_pd+mode, 8);
            // EC_WRITE_S32(domain_r_pd+tar_vel, 0xffff);
            // EC_WRITE_S32(domain_r_pd+max_torq, 0xf00);
            printf("Servo state 3\n");
            printf("     0x%04X  |\n", temp[0]);

        }
		
		//operation enabled

        // else if( (temp[0]&0x006f) == 0x0027)        // this servo is on
		// {
        //     // printf("Servo state 4\n");
        //     // printf("     0x%04X  |\n", temp[0]);
        //     t += 0.001; // increment time for simulation purposes
        //     move_value =  offsets; // simulate a sine wave position
        //     // move
        //     // move_value = 0;
        //     EC_WRITE_S32(domain_r_pd+tar_pos, move_value); // set target position
        //     EC_WRITE_U16(domain_r_pd+ctrl_word, 0x001f);

        //     printf("Offset: %d, Move value: %d, Actual pos: %d\n", offsets, move_value, actual_pos);
        //     // printf("Servo state 4\n");

        // }

        // temp[1]=EC_READ_U32(domain_w_pd + pos_act);
        // printf("Position actual value: %ld\n", temp[1]);

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
			
	 	
	    
    if (!(sc = ecrt_master_slave_config(master, servo1, servo1_code))) 
    {
	fprintf(stderr, "Failed to get slave1 configuration.\n");
        return -1;
    }   
    

#if SDO_ACCESS

    if (ecrt_slave_config_sdo8(sc, 0x6060, 0, 8))
    {
        return -1;
    }

    //     if (ecrt_slave_config_sdo32(sc, 0x3328, 0, 16000000))
    // {
    //     return -1;
    // }

    // if (ecrt_slave_config_sdo32(sc, 0x4602, 0, 1))
    // {
    //     return -1;
    // }




    // static uint8_t sdo_value;
    // size_t result_size;
    // uint32_t abort_code;
    // if (ecrt_master_sdo_upload(master, 0, 0x6060, 0, &sdo_value, sizeof(sdo_value), &result_size, &abort_code)) {
    //     printf("Failed to read SDO 0x6060. Abort code: 0x%08X\n", abort_code);
    //     ecrt_release_master(master);
    //     return -1;
    // }
    // printf("SDO 0x6060 value: %u, Result size: %zu\n", sdo_value, result_size);


#endif

 
// #if CONFIGURE_PDOS

//     printf("Configuring PDOs...\n");
	
//     if (ecrt_slave_config_pdos(sc, EC_END, slave_0_syncs)) 
//     {
//         fprintf(stderr, "Failed to configure 1st PDOs.\n");
//         return -1;
//     }
	
// #endif


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
		
	
	ecrt_slave_config_dc(sc,
        0x0300,
        1000000,4400000,0,0);  

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
        // while (1)
        // {
        //     /* code */
        // }
        

        
	ecrt_release_master(master);
	
    return 0;	
}