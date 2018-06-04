#ifndef RRT_GLOBAL_H
#define RRT_GLOBAL_H

#include <QtCore/qglobal.h>

#ifdef RRT_LIB
# define RRT_EXPORT Q_DECL_EXPORT
#else
# define RRT_EXPORT Q_DECL_IMPORT
#endif

#endif // RRT_GLOBAL_H
