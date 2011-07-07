#ifndef DLLHEADER_H_INCLUDED
#define DLLHEADER_H_INCLUDED

#ifdef DLL_EXPORT
# define EXPORT __declspec (dllexport)
#else
# define EXPORT
#endif

#include "std_msgs/String.h"

extern EXPORT int ROSSetup (char * argv, std::string masteruri, std::string localip);
extern EXPORT void ROSSend (std::string joint_name[20], float angle[20]);
extern EXPORT float ROSRec (std::string joint_name);
extern EXPORT bool ROSQuit ();

#endif