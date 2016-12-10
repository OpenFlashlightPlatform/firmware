#ifndef __BLUESUN_H__
#define __BLUESUN_H__

#ifdef __cplusplus
extern "C" {
#endif

//Sample the current through the load
double getShuntMilliAmps();

//Get the engine all set up
void initEngine();

#ifdef __cplusplus
}
#endif

#endif
