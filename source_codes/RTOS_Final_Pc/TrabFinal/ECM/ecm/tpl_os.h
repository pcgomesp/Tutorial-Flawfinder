/**
 * @file tpl_os.h
 *
 * @section File description
 *
 * This file gathers system call declarations.
 *
 * Generated from application test
 * Automatically generated by goil on Www Mmm dd hh:mm:ss yyyy
 * from root OIL file ecm.oil
 *
 * @section File informations
 *
 * $Date$
 * $Rev$
 * $Author$
 * $URL$
 */

#ifndef TPL_OS_H
#define TPL_OS_H


#ifdef __cplusplus
extern "C" {
#endif

#include "tpl_os_definitions.h"
#include "tpl_os_application_def.h"
#include "tpl_os_error.h"

/*--------------------------------------------------------------------------*
 * os system calls.
 * corresponding kernel implementation is in files
 * tpl_os_os_kernel.h and tpl_os_os_kernel.c
 * tpl_os_os.h and tpl_os_os.c
 *--------------------------------------------------------------------------*/
#include "tpl_os_os.h"

/*
 * GetActiveApplicationMode
 *
 * This service returns the current application mode. It may be used to 
 * write mode dependent code. 
 *
 * Return value:
 * The active application mode 
 */
FUNC(AppModeType, OS_CODE) GetActiveApplicationMode(void);

/*
 * ShutdownOS
 *
 * The user can call this system service to abort the overall system 
 * (e.g. emergency off). The operating system also calls this function 
 * internally, if it has reached an undefined internal state and is no 
 * longer ready to run. 
 *
 * error: The error that occured 
 */
FUNC(void, OS_CODE) ShutdownOS(
  CONST(StatusType, AUTOMATIC) error);


/*--------------------------------------------------------------------------*
 * interrupt system calls.
 * corresponding kernel implementation is in files
 * tpl_os_interrupt_kernel.h and tpl_os_interrupt_kernel.c
 *--------------------------------------------------------------------------*/
#include "tpl_os_interrupt.h"

/*
 * EnableAllInterrupts
 *
 * This service restores the state saved by DisableAllInterrupts. 
 */
FUNC(void, OS_CODE) EnableAllInterrupts(void);

/*
 * DisableAllInterrupts
 *
 * This service disables all interrupts for which the hardware supports 
 * disabling. The state before is saved for the EnableAllInterrupts call. 
 */
FUNC(void, OS_CODE) DisableAllInterrupts(void);

/*
 * ResumeAllInterrupts
 *
 * This service restores the recognition status of all interrupts saved 
 * by the SuspendAllInterrupts service. 
 */
FUNC(void, OS_CODE) ResumeAllInterrupts(void);

/*
 * SuspendAllInterrupts
 *
 * This service saves the recognition status of all interrupts and 
 * disables all interrupts for which the hardware supports disabling. 
 */
FUNC(void, OS_CODE) SuspendAllInterrupts(void);

/*
 * ResumeOSInterrupts
 *
 * This service restores the recognition status of interrupts saved by 
 * the SuspendOSInterrupts service. 
 */
FUNC(void, OS_CODE) ResumeOSInterrupts(void);

/*
 * SuspendOSInterrupts
 *
 * This service saves the recognition status of interrupts of category 2 
 * and disables the recognition of these interrupts. 
 */
FUNC(void, OS_CODE) SuspendOSInterrupts(void);


/*--------------------------------------------------------------------------*
 * task system calls.
 * corresponding kernel implementation is in files
 * tpl_os_task_kernel.h and tpl_os_task_kernel.c
 *--------------------------------------------------------------------------*/
#include "tpl_os_task.h"

/*
 * ActivateTask
 *
 * Activate a task 
 *
 * task_id: The identifier of the task to activate 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_LIMIT: Too many activations of <task_id> (Standard & Extended) 
 * E_OS_ID: <task_id> is invalid (Extended) 
 */
FUNC(StatusType, OS_CODE) ActivateTask(
  CONST(TaskType, AUTOMATIC) task_id);

/*
 * TerminateTask
 *
 * Terminate the calling task 
 *
 * Return value:
 * E_OS_RESOURCE: The calling task still occupies a resource (Extended) 
 * E_OS_CALLEVEL: Call at interrupt level (Extended) 
 */
FUNC(StatusType, OS_CODE) TerminateTask(void);

/*
 * ChainTask
 *
 * Terminate the calling task and activate task task_id 
 *
 * task_id: The identifier of the task to chain to 
 *
 * Return value:
 * E_OS_LIMIT: Too many activations of <task_id> (Standard & Extended) 
 * E_OS_ID: <task_id> is invalid (Extended) 
 * E_OS_RESOURCE: The calling task still occupies a resource (Extended) 
 * E_OS_CALLEVEL: Call at interrupt level (Extended) 
 */
FUNC(StatusType, OS_CODE) ChainTask(
  CONST(TaskType, AUTOMATIC) task_id);

/*
 * Schedule
 *
 * Call the scheduler 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_RESOURCE: The calling task still occupies a resource (Extended) 
 * E_OS_CALLEVEL: Call at interrupt level (Extended) 
 */
FUNC(StatusType, OS_CODE) Schedule(void);

/*
 * GetTaskID
 *
 * Get the id of the calling task 
 *
 * task_id: A pointer to the var where the identifier of the task will be 
 *           stored 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 */
FUNC(StatusType, OS_CODE) GetTaskID(
  VAR(TaskRefType, AUTOMATIC) task_id);

/*
 * GetTaskState
 *
 * Get the task state of a task 
 *
 * task_id: The identifier of the task 
 *
 * state: A pointer to the var where the state of the task will be stored 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_ID: <alarm_id> is invalid (Extended) 
 */
FUNC(StatusType, OS_CODE) GetTaskState(
  CONST(TaskType, AUTOMATIC) task_id,
  VAR(TaskStateRefType, AUTOMATIC) state);


/*--------------------------------------------------------------------------*
 * resource system calls.
 * corresponding kernel implementation is in files
 * tpl_os_resource_kernel.h and tpl_os_resource_kernel.c
 *--------------------------------------------------------------------------*/
#include "tpl_os_resource.h"

/*
 * GetResource
 *
 * Get resource res_id. As a result, the priority of the caller may be 
 * raised to the priority of the resource if the latter is higher 
 *
 * res_id: The id of the resource to get. 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_ID: <res_id> is invalid (Extended) 
 */
FUNC(StatusType, OS_CODE) GetResource(
  CONST(ResourceType, AUTOMATIC) res_id);

/*
 * ReleaseResource
 *
 * Release resource res_id. The priority of the caller returns to the 
 * priority it had before getting the resource 
 *
 * res_id: The id of the resource to release. 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_ID: <res_id> is invalid (Extended) 
 * E_OS_ACCESS: Attempt to get a resource which is already occupied by 
 *     any task or ISR, or the statically assigned priority of the 
 *     calling task or interrupt routine is higher than the calculated 
 *     ceiling priority (Extended) 
 */
FUNC(StatusType, OS_CODE) ReleaseResource(
  CONST(ResourceType, AUTOMATIC) res_id);


/*--------------------------------------------------------------------------*
 * alarm system calls.
 * corresponding kernel implementation is in files
 * tpl_os_alarm_kernel.h and tpl_os_alarm_kernel.c
 *--------------------------------------------------------------------------*/
#include "tpl_os_alarm.h"

/*
 * GetAlarmBase
 *
 * Get information about the underlying counter. 
 * See page 63 of the OSEK OS 2.2.3 specification. 
 *
 * alarm_id: The identifier of the alarm 
 *
 * info: A pointer to the AlarmBaseType data where the informations will 
 *        be stored 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_ID: <alarm_id> is invalid (Extended) 
 */
FUNC(StatusType, OS_CODE) GetAlarmBase(
  CONST(AlarmType, AUTOMATIC) alarm_id,
  VAR(AlarmBaseRefType, AUTOMATIC) info);

/*
 * GetAlarm
 *
 * Get the remaining number of ticks before the alarm expire. 
 * See page 63 of the OSEK OS 2.2.3 specification. 
 *
 * alarm_id: The identifier of the alarm 
 *
 * tick: A pointer to the TickType data where the remaining number of 
 *        ticks before the alarm expire will be stored 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_NOFUNC: Alarm <alarm_id> is not in use (Standard & Extended) 
 * E_OS_ID: <alarm_id> is invalid (Extended) 
 */
FUNC(StatusType, OS_CODE) GetAlarm(
  CONST(AlarmType, AUTOMATIC) alarm_id,
  VAR(TickRefType, AUTOMATIC) tick);

/*
 * SetRelAlarm
 *
 * SetRelAlarm starts alarm <alarm_id>. After <increment> ticks have 
 * elapsed, the task assigned to the alarm <alarm_id> is activated or the 
 * assigned event (only for extended tasks) is set or the alarm-callback 
 * routine is called. 
 * See page 63 of the OSEK OS 2.2.3 specification. 
 *
 * alarm_id: The identifier of the alarm 
 *
 * increment: Relative value in ticks. 
 *
 * cycle: Cycle value in case of a cyclic alarm. In case of a one shot 
 *         alarm, cycle shall be zero 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_NOFUNC: Alarm <alarm_id> is already in use (Standard & Extended) 
 * E_OS_ID: <alarm_id> is invalid (Extended) 
 * E_OS_VALUE: <increment> out of bounds (Extended) 
 * E_OS_VALUE: <cycle> out of bounds (Extended) 
 */
FUNC(StatusType, OS_CODE) SetRelAlarm(
  CONST(AlarmType, AUTOMATIC) alarm_id,
  CONST(TickType, AUTOMATIC) increment,
  CONST(TickType, AUTOMATIC) cycle);

/*
 * SetAbsAlarm
 *
 * SetAbsAlarm starts alarm <alarm_id>. At <start> ticks, the task 
 * assigned to the alarm <alarm_id> is activated or the assigned event 
 * (only for extended tasks) is set or the alarm-callback routine is 
 * called. 
 * See page 64 of the OSEK OS 2.2.3 specification. 
 *
 * alarm_id: The identifier of the alarm 
 *
 * start: Absolute value in ticks. 
 *
 * cycle: Cycle value in case of a cyclic alarm. In case of a one shot 
 *         alarm, cycle shall be zero 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_NOFUNC: Alarm <alarm_id> is already in use (Standard & Extended) 
 * E_OS_ID: <alarm_id> is invalid (Extended) 
 * E_OS_VALUE: <start> out of bounds (Extended) 
 * E_OS_VALUE: <cycle> out of bounds (Extended) 
 */
FUNC(StatusType, OS_CODE) SetAbsAlarm(
  CONST(AlarmType, AUTOMATIC) alarm_id,
  CONST(TickType, AUTOMATIC) start,
  CONST(TickType, AUTOMATIC) cycle);

/*
 * CancelAlarm
 *
 * CancelAlarm cancels a started alarm. 
 * See page 65 of the OSEK OS 2.2.3 specification. 
 *
 * alarm_id: The identifier of the alarm 
 *
 * Return value:
 * E_OK: No error (Standard & Extended) 
 * E_OS_NOFUNC: Alarm <alarm_id> is not in use (Standard & Extended) 
 * E_OS_ID: <alarm_id> is invalid (Extended) 
 */
FUNC(StatusType, OS_CODE) CancelAlarm(
  CONST(AlarmType, AUTOMATIC) alarm_id);


/*
 * Objects declared in the OIL/ARXML file
 */
DeclareApplicationMode(stdAppmode);
DeclareTask(interfaceCAN);
DeclareTask(interfaceProcessing);
DeclareTask(serialRPM);
DeclareResource(infoCAN);
DeclareResource(infoSerial);
DeclareAlarm(periodicAlarmCAN);
DeclareAlarm(periodicAlarmProcess);
DeclareAlarm(periodicAlarmSerial);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
#include "mcp_can.h"
#include "mcp_can_dfs.h"
#endif

/* TPL_OS_H */
#endif

/* End of file tpl_os.h */
