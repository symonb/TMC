#include <stdbool.h>
#include <stdio.h>
#include "scheduler/scheduler.h"
#include "config.h"
#include "IO/LED.h"
void taskFun1(timeUs_t t) {
	LED_Green_Toggle();
}
void taskFun2(timeUs_t t) {
	LED_Red_Toggle();
}

task_t tasks[TASK_COUNT] = {
	[TASK_SYSTEM] =
		{
		.taskName = "SYSTEM",
		.taskFun = taskSystem,
		.desiredPeriod = TASK_PERIOD_HZ(TASK_SYSTEM_PERIOD_HZ),
		.staticPriority = TASK_PRIORITY_HIGH
		},
	[TASK_T1] =
			{
			.taskName = "Task 1",
			.taskFun = taskFun1,
			.desiredPeriod = TASK_PERIOD_HZ(10),
			.staticPriority = TASK_PRIORITY_IDLE
			},

	[TASK_T2] =
		{
			.taskName = "Task 2",
			.taskFun = taskFun2,
			.desiredPeriod = TASK_PERIOD_HZ(2),
			.staticPriority = TASK_PRIORITY_IDLE
		}
};

void initTasks(void) {
	initScheduler();
	enableTask(TASK_T1, true);
	enableTask(TASK_T2, true);
}