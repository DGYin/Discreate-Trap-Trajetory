#include "Alg_TrapVelTraj.h"
#include "Alg_Series.h"
#include "Alg_Comparator.h"
/*
 * 【适用于浮点量的】生成梯形速度规划轨迹
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
		float ac, float de, float p0, float p1, float v0, float v1)
{
	float v02, v12, vc2; // 分别是v0, v1, vc的平方
	float const p = p1 - p0; // 计算位移差
	float const _2p = 2 * p; // 位移差的两倍
	bool const reversed = p < 0; // 判断轨迹方向是否反转
	// 如果加速率和减速率相同，无法生成轨迹
	if (ac == de)
	{
		return 0;
	}
	// 如果最大速度为负，取其绝对值
	if (vm < 0)
	{
		vm = -vm;
	}
	// 对起始速度和结束速度进行饱和处理，防止超过最大速度
	v0 = AlgComparator_Saturation(v0, -vm, +vm);
	v1 = AlgComparator_Saturation(v1, -vm, +vm);
	// 初始化轨迹上下文
	ctx->p0 = p0;
	ctx->p1 = p1;
	ctx->v0 = v0;
	ctx->v1 = v1;
	// 计算v0, v1, vc的速度平方
	v02 = v0 * v0;
	v12 = v1 * v1;
	// 计算最大速度平方vc2
	vc2 = (v12 * ac - v02 * de - _2p * ac * de) / (ac - de);
	// 如果vc2小于等于0，轨迹无法生成
	if (vc2 <= 0)
	{
		return 0;
	}
	// 如果vc2大于最大速度平方，则轨迹分为加速、匀速、减速三个阶段
	if (vc2 > vm * vm)
	{
		ctx->vc = reversed ? -vm : vm; // 设置最大速度
		ctx->ta = (ctx->vc - v0) / ac; // 计算加速时间
		ctx->t = (v1 - ctx->vc) / de; // 计算减速时间
		ctx->pa = p0 + ctx->v0 * ctx->ta + 0.5f * ac * ctx->ta * ctx->ta; // 加速段终点位置
		ctx->pd = p1 - ctx->vc * ctx->t - 0.5f * de * ctx->t * ctx->t; // 减速段起点位置
		ctx->td = ctx->ta + (ctx->pd - ctx->pa) / ctx->vc; // 匀速段开始时间
		ctx->t += ctx->td; // 总时间
	}
	// 如果vc2介于v02和v12之间，则轨迹只有加速段
	else if (vc2 > v02 && vc2 <= v12)
	{
		v12 = v02 + _2p * ac;
		if (v12 < 0)
		{
			return 0;
		}
		ctx->v1 = sqrt(v12);
		if (reversed)
		{
			ctx->v1 = -ctx->v1;
		}
		ctx->vc = ctx->v1;
		ctx->t = (ctx->v1 - v0) / ac;
		ctx->ta = ctx->t;
		ctx->td = ctx->t;
		ctx->pa = p0 + ctx->v0 * ctx->t + 0.5f * ac * ctx->t * ctx->t;
		ctx->pd = p1;
	}
	// 如果vc2小于等于v02且大于v12，则轨迹只有减速段
	else if (vc2 <= v02 && vc2 > v12)
	{
		v12 = v02 + _2p * de;
		if (v12 < 0)
		{
			return 0;
		}
		ctx->v1 = sqrt(v12);
		if (reversed)
		{
			ctx->v1 = -ctx->v1;
		}
		ctx->vc = ctx->v0;
		ctx->t = (ctx->v1 - v0) / de;
		ctx->ta = 0;
		ctx->td = 0;
		ctx->pa = p0;
		ctx->pd = p0;
	}
	// 如果vc2介于v02和v12之间，但轨迹没有匀速段，只有加速和减速段
	else /* acceleration, deceleration */
	{
		ctx->vc = sqrt(vc2);
		if (reversed)
		{
			ctx->vc = -ctx->vc;
		}
		ctx->t = (ctx->vc - v0) / ac;
		ctx->ta = ctx->t;
		ctx->td = ctx->t;
		ctx->pa = p0 + ctx->v0 * ctx->t + 0.5f * ac * ctx->t * ctx->t;
		ctx->t += (v1 - ctx->vc) / de;
		ctx->pd = ctx->pa;
	}
	// 保存加速率和减速率
	ctx->ac = ac;
	ctx->de = de;
	return ctx->t;
}

// 【适用于浮点量的】获取轨迹上任意时刻的位置（不适用于步进电机等！）
// @param ctx: 指向轨迹上下文的指针
// @param x: 当前时间点
// @return: 返回当前时间点上的位置
float AlgTrapVelTraj_GetPos(AlgVelProf_trapVelTrajCtx_t const *ctx, float x)
{
	if (x >= ctx->ta)
	{
		if (x < ctx->td) /* linear motion */
		{
			return ctx->pa + ctx->vc * (x - ctx->ta);
		}
		if (x < ctx->t) /* final blend */
		{
			x -= ctx->td;
			return ctx->pd + ctx->vc * x + 0.5f * ctx->de * x * x;
		}
		return ctx->p1;
	}
	if (x > 0) /* initial blend */
	{
		return ctx->p0 + ctx->v0 * x + 0.5f * ctx->ac * x * x;
	}
	return ctx->p0;
}

// 【适用于浮点量的】获取轨迹上任意时刻的速度
// @param ctx: 指向轨迹上下文的指针
// @param x: 当前时间点
// @return: 返回当前时间点上的速度
float AlgTrapVelTraj_GetVel(AlgVelProf_trapVelTrajCtx_t const *ctx, float x)
{
	if (x >= ctx->ta)
	{
		if (x < ctx->td) /* linear motion */
		{
			return ctx->vc;
		}
		if (x < ctx->t) /* final blend */
		{
			return ctx->vc + ctx->de * (x - ctx->td);
		}
		return ctx->v1;
	}
	if (x > 0) /* initial blend */
	{
		return ctx->v0 + ctx->ac * x;
	}
	return ctx->v0;
}

// 【适用于浮点量的】获取轨迹上任意时刻的加速度
// @param ctx: 指向轨迹上下文的指针
// @param x: 当前时间点
// @return: 返回当前时间点上的加速度
float AlgTrapVelTraj_GetAcc(AlgVelProf_trapVelTrajCtx_t const *ctx, float x)
{
	if (x < ctx->ta)
	{
		if (x >= 0) /* initial blend */
		{
			return ctx->ac;
		}
	}
	else if (x >= ctx->td)
	{
		if (x <= ctx->t) /* final blend */
		{
			return ctx->de;
		}
	}
	return 0; /* linear motion */
}

/*
 * 【整形改进后的】梯形轨迹生成
 */
uint32_t AlgTrapVelTraj_TrailGenIntImproved(
		AlgVelProf_trapVelTrajCtxIntImproved_t *ctxImproved, int32_t vm,
		int32_t ac, int32_t de, int32_t p0, int32_t p1, int32_t v0, int32_t v1)
{
	uint32_t ret;
	memset(ctxImproved, 0, sizeof(AlgVelProf_trapVelTrajCtxIntImproved_t));
	float floatTime = AlgTrapVelTraj_TrailGen(&ctxImproved->ctx, vm, ac, de, p0,
			p1, v0, v1);
	if (round(floatTime) - floor(floatTime) == 1) // 五入，可以直接使用 AlgTrapVelTraj_GetVelIntImproved
		ret = round(floatTime);
	else
	{
		ctxImproved->roundDown = true; // 四舍，要特殊处理
	}
	ctxImproved->endTick = ceil(floatTime);
	// 未经离散化补偿的运行结果，相减得到待补偿值
	for (int i = 0; i < ctxImproved->endTick; i++)
	{
		ctxImproved->intTrajRawOutput = ctxImproved->intTrajRawOutput
				+ AlgTrapVelTraj_GetVelIntImproved(&ctxImproved->ctx, i);
	}
	ctxImproved->intTrajCompensation = (p1 - p0)
			- ctxImproved->intTrajRawOutput;
	// 把补偿值分散到加速和减速阶段
	ctxImproved->accCompValue = (ceil(ctxImproved->intTrajCompensation / 2.f));
	ctxImproved->decCompValue = (floor(ctxImproved->intTrajCompensation / 2.f));
	ret = ceil(floatTime);
	return ret;
}

/*
 * 【整形改进后的】获取轨迹上任意时刻的速度
 */
int32_t AlgTrapVelTraj_GetVelIntImproved(
		AlgVelProf_trapVelTrajCtxIntImproved_t *ctxImproved, uint32_t x)
{
	if (x > 0 && x < ctxImproved->endTick) // 掐头去尾，防止影响首尾速度
	{
		int32_t timeSpan = (ctxImproved->endTick - 2);
		int32_t avgComp = ctxImproved->intTrajCompensation / timeSpan; // 平均分散补偿
		int32_t remainedComp = ctxImproved->intTrajCompensation % timeSpan; // 剩下的就尽量放到加速阶段，从第一位开始每时刻都补偿 1
		int32_t sign;
		if (remainedComp != 0)
			sign = remainedComp / abs(remainedComp); // sign 函数
		if (x <= abs(remainedComp))
			return round(
					AlgTrapVelTraj_GetVel(&ctxImproved->ctx, (float) x - 0.5f))
					+ avgComp + sign;
		else
			return round(
					AlgTrapVelTraj_GetVel(&ctxImproved->ctx, (float) x - 0.5f))
					+ avgComp;
	}
	if (x == 0)
		return ctxImproved->ctx.v0;
	else if (ctxImproved->roundDown && x == ctxImproved->endTick) // 四舍，带入是没有结果的，直接返回 v1
		return (int32_t) ctxImproved->ctx.v1;
}
