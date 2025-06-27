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

#define ecc  		0,6
#define ecc_code 	0x00000083, 0x000000a6

int demlanlap = 0;
double t = 0;
/***************************************************************************/

//signal to turn off servo on state
static unsigned int servo_flag =0;
static unsigned int deactive;

static unsigned int Iread;
static unsigned int Owrite;
static signed long IOvalue;
static uint8_t shift_index = 0;
static signed int loop_counter; 

//rx pdo entry 
const static ec_pdo_entry_reg_t domain_r_regs[] = 
{
    {ecc,ecc_code,0x7003,0x01,&Owrite},  
    {}
};

//tx pdo entry
const static ec_pdo_entry_reg_t domain_w_regs[] = 
{
    {ecc,ecc_code,0x6003,0x01,&Iread},      
    {}
};




float move_value = 0;
static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};


/*****************************************************************************/

#if PDO_SETTING
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


        if (loop_counter) 
		{
            loop_counter--;
        } 	
		else 
		{ // do this at 1 Hz
            loop_counter = 30;
            IOvalue = EC_READ_U32(domain_w_pd + Iread); // read the value from the process data
            // EC_WRITE_U32(domain_r_pd + Owrite, 0xf0f0f0f0); // read the value from the process data
            printf("IOvalue: %ld\n", IOvalue);

            uint32_t pattern = ((1U << WINDOW) - 1) << (WIDTH - WINDOW - shift_index);

            // Write the pattern to the process data
            EC_WRITE_U32(domain_r_pd + Owrite, pattern);

            // Update the shift index
            shift_index++;
            if (shift_index > WIDTH - WINDOW) {
                shift_index = 0;  // Reset to loop back
        }
        }


        



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