#define _POSIX_C_SOURCE 200112L
#include <stdio.h>
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
#include <sched.h> /* sched_setscheduler() */

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING

#define CONFIGURE_PDOS 1
#define PDO_SETTING	1

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

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_domain_t *domain2 = NULL;
static ec_domain_state_t domain2_state = {};

static ec_slave_config_t *sc1  = NULL;
static ec_slave_config_t *sc2  = NULL;

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;
static uint8_t *domain2_pd = NULL;

// PDO entry offsets
static unsigned int off_controlword1 = 0;
static unsigned int off_target_position1 = 0;
static unsigned int off_statusword1 = 0;
static unsigned int off_position_actual_value1 = 0;

// static unsigned int off_touch_probe_function = 0;
// static unsigned int off_physical_outputs = 0;
// static unsigned int off_error_code = 0;
// static unsigned int off_torque_actual_value = 0;
// static unsigned int off_following_error = 0;
// static unsigned int off_touch_probe_status = 0;
// static unsigned int off_touch_probe_pos1 = 0;
// static unsigned int off_touch_probe_pos2 = 0;
// static unsigned int off_digital_inputs = 0;

static unsigned int off_controlword2 = 0;
static unsigned int off_target_position2 = 0;
static unsigned int off_statusword2 = 0;
static unsigned int off_position_actual_value2 = 0;

static signed long temp[8]={};



#define servo1  		0,0
#define servo1_code 	0x00000083, 0x00000005

#define servo2  		0,1
#define servo2_code 	0x00000083, 0x00000007

// offsets for PDO entries
float move_value1 = 0;
float move_value2 = 0;
static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};


ec_pdo_entry_reg_t domain1_regs[] = {
    {servo1, servo1_code, 0x6040, 0x00, &off_controlword1},             // Controlword
    {servo1, servo1_code, 0x607a, 0x00, &off_target_position1},         // Target Position
    // {servo1_code, servo1, 0x60b8, 0x00, &off_touch_probe_function},    // Touch Probe Function
    // {servo1_code, servo1, 0x60fe, 0x01, &off_physical_outputs},        // Physical Outputs

    // {servo1_code, servo1, 0x603f, 0x00, &off_error_code},              // Error code
    {servo1, servo1_code, 0x6041, 0x00, &off_statusword1},              // Statusword
    // {servo1_code, servo1, 0x6064, 0x00, &off_position_actual_value},   // Position Actual Value
    // {servo1_code, servo1, 0x6077, 0x00, &off_torque_actual_value},     // Torque Actual Value
    // {servo1_code, servo1, 0x60f4, 0x00, &off_following_error},         // Following Error
    // {servo1_code, servo1, 0x60b9, 0x00, &off_touch_probe_status},      // Touch Probe Status
    // {servo1_code, servo1, 0x60ba, 0x00, &off_touch_probe_pos1},        // Touch Probe Pos1
    // {servo1_code, servo1, 0x60bc, 0x00, &off_touch_probe_pos2},        // Touch Probe Pos2
    // {servo1_code, servo1, 0x60fd, 0x00, &off_digital_inputs},          // Digital Inputs
    {}
};

ec_pdo_entry_reg_t domain2_regs[] = {
    {servo2, servo2_code, 0x6040, 0x00, &off_controlword2},             // Controlword
    {servo2, servo2_code, 0x607a, 0x00, &off_target_position2},         // Target Position
    // {servo1_code, servo1, 0x60b8, 0x00, &off_touch_probe_function},    // Touch Probe Function
    // {servo1_code, servo1, 0x60fe, 0x01, &off_physical_outputs},        // Physical Outputs

    // {servo1_code, servo1, 0x603f, 0x00, &off_error_code},              // Error code
    {servo2, servo2_code, 0x6041, 0x00, &off_statusword2},              // Statusword
    // {servo1_code, servo1, 0x6064, 0x00, &off_position_actual_value},   // Position Actual Value
    // {servo1_code, servo1, 0x6077, 0x00, &off_torque_actual_value},     // Torque Actual Value
    // {servo1_code, servo1, 0x60f4, 0x00, &off_following_error},         // Following Error
    // {servo1_code, servo1, 0x60b9, 0x00, &off_touch_probe_status},      // Touch Probe Status
    // {servo1_code, servo1, 0x60ba, 0x00, &off_touch_probe_pos1},        // Touch Probe Pos1
    // {servo1_code, servo1, 0x60bc, 0x00, &off_touch_probe_pos2},        // Touch Probe Pos2
    // {servo1_code, servo1, 0x60fd, 0x00, &off_digital_inputs},          // Digital Inputs
    {}
};


#if PDO_SETTING

 ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60b8, 0x00, 16}, /* Touch probe function */
    {0x60fe, 0x01, 32}, /* Digital outputs */
    {0x603f, 0x00, 16}, /* Error code */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x60f4, 0x00, 32}, /* Following error actual value */
    {0x60b9, 0x00, 16}, /* Touch probe status */
    {0x60ba, 0x00, 32}, /* Touch probe pos1 pos value */
    {0x60bc, 0x00, 32}, /* Touch probe pos2 pos value */
    {0x60fd, 0x00, 32}, /* Digital inputs */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1701, 4, slave_0_pdo_entries + 0}, /* 258th receive PDO Mapping */
    {0x1b01, 9, slave_0_pdo_entries + 4}, /* 258th transmit PDO Mapping */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

#endif

/****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

void endsignal(int sig)
{	
	// servo_flag = 1;
	signal( SIGINT , SIG_DFL );
}
/****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;
    

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

void check_domain2_state(void)
{
    ec_domain_state_t ds;
    

    ecrt_domain_state(domain2, &ds);

    if (ds.working_counter != domain2_state.working_counter)
        printf("Domain2: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain2: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/****************************************************************************/

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
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

    while(1) {
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

        // Write application time to master
        //
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        //
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns) {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns) {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns) {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns) {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns) {
            exec_min_ns = exec_ns;
        }
#endif

        // receive process data
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        ecrt_domain_process(domain2);

        // check process data state (optional)
        // check_domain1_state();
        // check_domain2_state();

        if (counter) {
            counter--;
        } else { // do this at 1 Hz
            counter = FREQUENCY;

            // check for master state (optional)
            check_master_state();

#ifdef MEASURE_TIMING
            // output timing stats
            printf("period     %10u ... %10u\n",
                    period_min_ns, period_max_ns);
            printf("exec       %10u ... %10u\n",
                    exec_min_ns, exec_max_ns);
            printf("latency    %10u ... %10u\n",
                    latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
#endif

            // calculate new process data
            blink = !blink;
        }

        // Read process data domain1 
        uint16_t status1 = EC_READ_U16(domain1_pd + off_statusword1);
        temp[1] = status1;

        // write process data
        if ((temp[1] & 0x004f) == 0x0040) {
            // Ready to switch on
            EC_WRITE_U16(domain1_pd + off_controlword1, 0x0006);
        }
        else if ((temp[1] & 0x006f) == 0x0021) {
            // Switched on
            EC_WRITE_U16(domain1_pd + off_controlword1, 0x0007);
        }
        else if ((temp[1] & 0x006f) == 0x0023) {
            // Operation enabled, write initial target
            EC_WRITE_U16(domain1_pd + off_controlword1, 0x000F);
            EC_WRITE_S32(domain1_pd + off_target_position1, 0);
        }


        //
        // Read process data domain2
        uint16_t status2 = EC_READ_U16(domain2_pd + off_statusword2);
        temp[2] = status2;

        // write process data
        if ((temp[2] & 0x004f) == 0x0040) {
            // Ready to switch on
            EC_WRITE_U16(domain2_pd + off_controlword2, 0x0006);
        }
        else if ((temp[2] & 0x006f) == 0x0021) {
            // Switched on
            EC_WRITE_U16(domain2_pd + off_controlword2, 0x0007);
        }
        else if ((temp[2] & 0x006f) == 0x0023) {
            // Operation enabled, write initial target
            EC_WRITE_U16(domain2_pd + off_controlword2, 0x000F);
            EC_WRITE_S32(domain2_pd + off_target_position2, 0);
        }


        if (((temp[1] & 0x006f) == 0x0027) & ((temp[2] & 0x006f) == 0x0027)) {
            // Operation fully enabled — send motion
            move_value1 += 5000;
            move_value2 = -move_value1;
            printf("%f\n", move_value1);
            EC_WRITE_S32(domain1_pd + off_target_position1, move_value1);
            EC_WRITE_U16(domain1_pd + off_controlword1, 0x001F);

            EC_WRITE_S32(domain2_pd + off_target_position2, move_value2);
            EC_WRITE_U16(domain2_pd + off_controlword2, 0x001F);
        }

        if (sync_ref_counter) {
            sync_ref_counter--;
        } else {
            sync_ref_counter = 1; // sync every cycle

            clock_gettime(CLOCK_TO_USE, &time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
        }
        ecrt_master_sync_slave_clocks(master);

        // send process data
        ecrt_domain_queue(domain1);
        ecrt_domain_queue(domain2);
        ecrt_master_send(master);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
    }
}

/****************************************************************************/

int main(int argc, char **argv)
{

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }
    
    master = ecrt_request_master(0);
    if (!master)
        return -1;
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
        return -1;
    domain2 = ecrt_master_create_domain(master);
    if (!domain2)
        return -1;

    // Create configuration for slave
    sc1 = ecrt_master_slave_config(master, servo1, servo1_code);
    if (!sc1){
        fprintf(stderr, "Failed to get Omron servo1 configuration.\n");
        return -1;
    }

    sc2 = ecrt_master_slave_config(master, servo2, servo2_code);
    if (!sc2){
        fprintf(stderr, "Failed to get Omron servo2 configuration.\n");
        return -1;
    }

    if (ecrt_slave_config_sdo8(sc1, 0x6060, 0, 8) || ecrt_slave_config_sdo8(sc2, 0x6060, 0, 8))
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc1, 0x3328, 0, 16000000))
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc2, 0x3328, 0, 16000000))
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc1, 0x6065, 0, 0xFFFFFFFF))
    {
        return -1;
    }

    if (ecrt_slave_config_sdo32(sc2, 0x6065, 0, 0xFFFFFFFF))
    {
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc1, 0x3329, 0, 99))
    {
        return -1;
    }

    if (ecrt_slave_config_sdo16(sc2, 0x3329, 0, 99))
    {
        return -1;
    }

#if CONFIGURE_PDOS

    printf("Configuring PDOs...\n");
	
    if (ecrt_slave_config_pdos(sc1, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure 1st PDOs.\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc2, EC_END, slave_0_syncs)) 
    {
        fprintf(stderr, "Failed to configure 2st PDOs.\n");
        return -1;
    }
	
#endif

    // Register PDO entries
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "Failed to register PDO 1 entries.\n");
        return -1;
    }
    if (ecrt_domain_reg_pdo_entry_list(domain2, domain2_regs)) {
        fprintf(stderr, "Failed to register PDO 2 entries.\n");
        return -1;
    }

    // configure SYNC signals for this slave
    ecrt_slave_config_dc(sc1, 0x0300, PERIOD_NS, 4400000, 0, 0);
    ecrt_slave_config_dc(sc2, 0x0300, PERIOD_NS, 4400000, 0, 0);

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    if (!(domain2_pd = ecrt_domain_data(domain2))) {
        return -1;
    }

    /* Set priority */

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
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

/****************************************************************************/
