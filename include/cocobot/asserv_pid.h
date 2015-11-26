#ifndef COCOBOT_ASSERV_PID_H
#define COCOBOT_ASSERV_PID_H

//internal structure
typedef struct
{
  float kp;
  float kd;
  float ki;

  float max_integral;
  float max_error_for_integration;

  float input;
  float feedback;
  float last_error;
  float integral;

  float output;
} cocobot_asserv_pid_t;


/* Initialization of the pid module. Need to be called before any other action 
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 */
void cocobot_asserv_pid_init(cocobot_asserv_pid_t * pid);

/* Reset pid for hard stop (opponent detected)
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 */
void cocobot_asserv_pid_reset(cocobot_asserv_pid_t * pid);

/* Set the proportionnal coef
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 *  - kp: coef
 */
void cocobot_asserv_pid_set_kp(cocobot_asserv_pid_t * pid, float kp);

/* Set the derivate coef
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 *  - kd: coef
 */
void cocobot_asserv_pid_set_kd(cocobot_asserv_pid_t * pid, float kd);

/* Set the integral coef
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 *  - ki: coef
 */
void cocobot_asserv_pid_set_ki(cocobot_asserv_pid_t * pid, float ki);

/* Set the integral saturation value
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 *  - max_integral: maximal integral value allowed
 */
void cocobot_asserv_pid_set_max_integral(cocobot_asserv_pid_t * pid, float max_integral);

/* Set the maximal error value before integral is disabled and resetted
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 *  - max_error_for_integration: maximal error value
 */
void cocobot_asserv_pid_set_max_error_for_integration(cocobot_asserv_pid_t * pid, float max_error_for_integration);

/* Set the PID set point
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 *  - input: regulation set point
 */
void cocobot_asserv_pid_set_input(cocobot_asserv_pid_t * pid, float input);

/* Set the feedback
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 *  - input: regulation feedback
 */
void cocobot_asserv_pid_set_feedback(cocobot_asserv_pid_t * pid, float feedback);

/* Get the output of the regulation
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 * Return:
 *  Output of the PID
 */
float cocobot_asserv_pid_get_output(cocobot_asserv_pid_t * pid);

/* Run the computation
 * Argument:
 *  - pid: a valid cocobot_asserv_pid_t pointer
 */
void cocobot_asserv_pid_compute(cocobot_asserv_pid_t * pid);

#endif// COCOBOT_ASSERV_PID_H

