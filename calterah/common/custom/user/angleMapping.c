#include "angleMapping.h"
#include <math.h>
#include "cfg.h"

void MapperSort(Angle *mapper, int left, int right);

int setAngleValue(int txCh, float measureVal, float mappingVal)
{
	int res = RADAR_SUCCESS;
	const Angle *pMap = radar_config_using->angleMapper[txCh];

	if (measureVal < pMap[0].measured || measureVal > pMap[ANG_NUM - 1].measured)
	{
		return RADAR_FAILED;
	}

	for (int i = 0; i < ANG_NUM; i++)
	{
		if (fabsf(measureVal - pMap[i].measured) < 1e-6)
		{
			setCfgAngleMapVal(txCh, i, mappingVal);
			break;
		}

		if (measureVal < pMap[i].measured)
		{
			res = RADAR_FAILED;
			break;
		}
	}
	return res;
}

void AngleMapping(int txCh, float measureVal, float *mappingVal)
{
	int indexMin = -1;
	int indexMax = -1;
	int i = 0;
	const Angle *pMap = radar_config_using->angleMapper[txCh];

	if (radar_config_using->angleMapperEn == 0 || measureVal < pMap[0].measured || measureVal > pMap[ANG_NUM - 1].measured)
	{
		*mappingVal = measureVal;
		return;
	}

	for (i = 0; i < ANG_NUM; ++i)
	{
		if (measureVal <= pMap[i].measured)
		{
			break;
		}
	}

	if (fabsf(measureVal - pMap[i].measured) < 1e-6)
	{
		*mappingVal = pMap[i].real;
		return;
	}

	indexMax = i == 0 ? 1 : i;
	indexMin = indexMax - 1;

	*mappingVal = (pMap[indexMax].real - pMap[indexMin].real) / (pMap[indexMax].measured - pMap[indexMin].measured)
		        * (measureVal - pMap[indexMin].measured) + pMap[indexMin].real;

}

void MapperSort(Angle *mapper, int left, int right)
{
	if (left >= right)/*如果左边索引大于或者等于右边的索引就代表已经整理完成一个组了*/
	{
		return;
	}
	int i = left;
	int j = right;
	float key = mapper[left].measured;
	Angle target = mapper[left];
	while (i < j)                               /*控制在当组内寻找一遍*/
	{

		while (i < j && key <= mapper[j].measured)
			/*而寻找结束的条件就是，1，找到一个小于或者大于key的数（大于或小于取决于你想升
			序还是降序）2，没有符合条件1的，并且i与j的大小没有反转*/
		{
			j--;/*向前寻找*/
		}

		//a[i] = a[j];		
        mapper[i] = mapper[j];
		/*找到一个这样的数后就把它赋给前面的被拿走的i的值（如果第一次循环且key是
		a[left]，那么就是给key）*/

		while (i < j && key >= mapper[i].measured)
			/*这是i在当组内向前寻找，同上，不过注意与key的大小关系停止循环和上面相反，
			因为排序思想是把数往两边扔，所以左右两边的数大小与key的关系相反*/
		{
			i++;
		}

		//a[j] = a[i];
        mapper[j] = mapper[i];
	}

	mapper[i] = target;;/*当在当组内找完一遍以后就把中间数key回归*/
	MapperSort(mapper, left, i - 1);/*最后用同样的方式对分出来的左边的小组进行同上的做法*/
	MapperSort(mapper, i + 1, right);/*用同样的方式对分出来的右边的小组进行同上的做法*/
					   /*当然最后可能会出现很多分左右，直到每一组的i = j 为止*/
}
