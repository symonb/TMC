#include <string.h>
#include <stdbool.h>
#include "scheduler.h"
#include "time/time.h"

task_t* taskQueue[TASK_COUNT];
task_t* currentTask = NULL;
static uint8_t taskQueueSize = 0;
static uint8_t taskQueuePos = 0;

static uint32_t totalWaitingTasks;
static uint32_t totalWaitingTasksSamples;
uint16_t averageSystemLoadPercent;

void taskSystem(timeUs_t currentTime) {
	if (totalWaitingTasksSamples > 0) {
		averageSystemLoadPercent =  100*totalWaitingTasks / totalWaitingTasksSamples;
		totalWaitingTasksSamples = 0;
		totalWaitingTasks = 0;
	}
}

static void clearQueue(void) {
	memset(taskQueue, 0, sizeof(taskQueue));
	taskQueuePos = 0;
	taskQueueSize = 0;
}

static bool isInQueue(task_t* task) {
	for (uint8_t i = 0; i < taskQueueSize; i++)
		if (taskQueue[i] == task)
			return true;
	return false;
}

static bool addToQueue(task_t* task) {
	if (isInQueue(task) || taskQueueSize >= TASK_COUNT)	//make sure that we have place
		return false;
	for (uint8_t i = 0; i <= taskQueueSize; i++) {
		if (taskQueue[i] == NULL)
		{
			memmove(&taskQueue[i + 1], &taskQueue[i], sizeof(task)*(taskQueueSize - i));
			taskQueue[i] = task;
			taskQueueSize++;
			return true;
		}
	}
	return false;
}

static bool queueRemove(task_t* task)
{
	for (int i = 0; i < taskQueueSize; i++) {
		if (taskQueue[i] == task) {
			memmove(&taskQueue[i], &taskQueue[i + 1], sizeof(task) * (taskQueueSize - i));
			taskQueueSize--;
			return true;
		}
	}
	return false;
}

static task_t* queueFirst(void) {
	taskQueuePos = 0;
	return taskQueue[0];
}

static task_t* queueNext(void) {
	return taskQueue[++taskQueuePos];
}

void enableTask(taskID_e taskID, bool enabled) 
{
	if (taskID < TASK_COUNT) {
		task_t* task =  &tasks[taskID];
		enabled ? addToQueue(task) : queueRemove(task);
	}
	
}	
void initScheduler(void) {
	clearQueue();
	enableTask(TASK_SYSTEM, true);
}
void scheduler(void) {
	const timeUs_t currentTime = micros();
	task_t* selectedTask = NULL;
	timeUs_t timeToNextRealTimeTask = 0;
	uint16_t selectedTaskDynamicPriority = 0;
	//Check if there is some real time task todo//
	for (task_t* task = queueFirst(); task != NULL && task->staticPriority >=TASK_PRIORITY_REALTIME; task = queueNext()) {
		const timeUs_t nextAt = task->lastExecution + task->desiredPeriod;
		if ((int32_t)(currentTime - nextAt) >= 0)
			timeToNextRealTimeTask = 0;
		else
			timeToNextRealTimeTask = nextAt - currentTime;
	}
	const bool realTimeTaskNotPending = (timeToNextRealTimeTask > 0);

	uint8_t waitingTasks = 0;

	for (task_t* task = queueFirst(); task != NULL; task = queueNext()) {
		//event-driven task

		if (task->checkFun) {
			const timeUs_t currentTimeBeforeCheckFuncCallUs = micros();
			if (task->dynamicPriority > 0) {
				task->ageCycles = (currentTime - task->lastExecution) / task->desiredPeriod;
				task->dynamicPriority = 1 + task->staticPriority * task->ageCycles;
				waitingTasks++;
			}
			else if (task->checkFun(currentTimeBeforeCheckFuncCallUs, currentTimeBeforeCheckFuncCallUs - task->lastExecution)) {
				task->lastSignaled = currentTimeBeforeCheckFuncCallUs;
				task->ageCycles = 1;
				task->dynamicPriority = 1 + task->staticPriority;;
				waitingTasks++;
			}
			else
			{
				task->ageCycles = 0;
			}
		}
		//time-driven task
		else {
			task->ageCycles = (currentTime - task->lastExecution) / task->desiredPeriod;
			if (task->ageCycles > 0) {
				task->dynamicPriority = 1 + task->staticPriority * task->ageCycles;
				waitingTasks++;
			}

		}

		if (task->dynamicPriority > selectedTaskDynamicPriority) {
			const bool canBeChoosen = (realTimeTaskNotPending) || (task->staticPriority >= TASK_PRIORITY_REALTIME)||(task->ageCycles>1);

			if (canBeChoosen) {
				selectedTaskDynamicPriority = task->dynamicPriority;
				selectedTask = task;
			}
		}
	}
	totalWaitingTasksSamples++;
	totalWaitingTasks += waitingTasks;

	currentTask = selectedTask;
	if (selectedTask) {
		const timeUs_t timeBeforeTask = micros();
		selectedTask->taskFun(micros());
		const timeUs_t timeAfterTask = micros();
		timeUs_t dt = timeAfterTask - timeBeforeTask;
		selectedTask->totalExecutionTime += dt;
		selectedTask->maxExecutionTime = MAX(dt, selectedTask->maxExecutionTime);
		selectedTask->movingSumExTime += dt - selectedTask->movingSumExTime / 32;
		selectedTask->lastExecution = currentTime;
		selectedTask->dynamicPriority = 0;
	}
}