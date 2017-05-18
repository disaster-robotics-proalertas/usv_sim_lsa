#ifndef SIMULATORCONFIG_H
#define SIMULATORCONFIG_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 M_PI/2
#endif
#ifndef M_PI_4
#define M_PI_4 M_PI/4
#endif

#include <osg/Notify>

#ifndef OSG_FATAL
#define OSG_FATAL osg::notify(osg::FATAL)
#endif

#ifndef OSG_ERROR
#define OSG_ERROR osg::notify(osg::ERROR)
#endif

#ifndef OSG_WARN
#define OSG_WARN osg::notify(osg::WARN)
#endif

#ifndef OSG_INFO
#define OSG_INFO osg::notify(osg::INFO)
#endif

#ifndef OSG_DEBUG
#define OSG_DEBUG osg::notify(osg::DEBUG_INFO)
#endif

//Build ROS interfaces
/* #undef BUILD_ROS_INTERFACES */

//Build Bullet physics
/* #undef BUILD_BULLET_PHYSICS */

//UWSim root folder
#define UWSIM_ROOT_PATH "/home/lsa/usv_sim_lsa/install_isolated/share/uwsim"

//Operating system
/* #undef WIN32 */
#define UNIX 1
/* #undef APPLE */

#endif

