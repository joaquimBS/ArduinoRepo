#include "PeriodicManager.h"

void PeriodicManager::run()
{
	(*p[0])();
}

void PeriodicManager::addPeriodicTask(void (*task)())
{
	
	p[0] = task;
}
