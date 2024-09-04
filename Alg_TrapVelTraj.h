#ifndef ALG_TRAP_VEL_TRAJ_H
#define ALG_TRAP_VEL_TRAJ_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

typedef struct
{
	float t; //!< total duration
	float p0; //!< initial position
	float p1; //!< final position
	float v0; //!< initial velocity
	float v1; //!< final velocity
	float vc; //!< constant velocity
	float ta; //!< time before constant velocity
	float td; //!< time after constant velocity
	float pa; //!< position before constant velocity
	float pd; //!< position after constant velocity
	float ac; //!< acceleration before constant velocity
	float de; //!< acceleration after constant velocity

} AlgVelProf_trapVelTrajCtx_t;

typedef struct
{
	AlgVelProf_trapVelTrajCtx_t ctx;
	int32_t intTrajRawOutput;
	int32_t intTrajCompensation;
	int32_t accCompValue;
	int32_t decCompValue;
	uint32_t endTick;
	bool roundDown;
} AlgVelProf_trapVelTrajCtxIntImproved_t;
/*
 * 生成梯形速度规划轨迹
 * @param ctx: 指向轨迹上下文的指针，用于保存轨迹数据
 * @param vm: 最大速度，正值表示正向最大速度，负值表示反向最大速度
 * @param ac: 加速率，单位时间内速度增加量。
 * @param de: 减速率，单位时间内速度减少量。应该是负数
 * @param p0: 起始位置
 * @param p1: 结束位置，如果p1<p0，则轨迹方向反转
 * @param v0: 起始速度
 * @param v1: 结束速度
 * @return: 返回整个轨迹所需的时间，如果无法生成轨迹则返回0
 */
float AlgTrapVelTraj_TrailGen(AlgVelProf_trapVelTrajCtx_t *ctx, float vm,
		float ac, float de, float p0, float p1, float v0, float v1);
float AlgTrapVelTraj_GetPos(AlgVelProf_trapVelTrajCtx_t const *ctx, float x);
float AlgTrapVelTraj_GetVel(AlgVelProf_trapVelTrajCtx_t const *ctx, float x);
float AlgTrapVelTraj_GetAcc(AlgVelProf_trapVelTrajCtx_t const *ctx, float x);
uint32_t AlgTrapVelTraj_TrailGenIntImproved(
		AlgVelProf_trapVelTrajCtxIntImproved_t *ctxImproved, int32_t vm,
		int32_t ac, int32_t de, int32_t p0, int32_t p1, int32_t v0, int32_t v1);
int32_t AlgTrapVelTraj_GetVelIntImproved(
		AlgVelProf_trapVelTrajCtxIntImproved_t *ctxImproved, uint32_t x);

#endif
