#include "oled.h"
#include "i2c.h"
#include "oledfont.h" //头文件
#include <stdlib.h>

// OLED初始化命令数组
uint8_t CMD_Data[] = {
	0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,
	0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12,
	0xD8, 0x30, 0x8D, 0x14, 0xAF}; // 初始化命令

/**
 * @brief  内部辅助函数：计算10的幂
 */
static uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while (n--)
		result *= m;
	return result;
}

// 写入OLED初始化命令
void WriteCmd(void)
{
	uint8_t i = 0;
	for (i = 0; i < 27; i++)
	{
		HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, CMD_Data + i, 1, 0x100);
	}
}

// 向设备写命令数据
void OLED_WR_CMD(uint8_t cmd)
{
	HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0x100);
}

// 向设备写显示数据
void OLED_WR_DATA(uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, &data, 1, 0x100);
}

// 初始化oled屏幕
void OLED_Init(void)
{
	HAL_Delay(200);
	WriteCmd();
}

// 清屏函数
void OLED_Clear(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		OLED_WR_CMD(0xb0 + i); // 设置页地址
		OLED_WR_CMD(0x00);	   // 设置显示位置低地址
		OLED_WR_CMD(0x10);	   // 设置显示位置高地址
		for (n = 0; n < 128; n++)
			OLED_WR_DATA(0); // 写入空数据，清除显示
	}
}

// 开启OLED显示
void OLED_Display_On(void)
{
	OLED_WR_CMD(0X8D); // 设置DCDC命令
	OLED_WR_CMD(0X14); // DCDC开启
	OLED_WR_CMD(0XAF); // 显示开启
}

// 关闭OLED显示
void OLED_Display_Off(void)
{
	OLED_WR_CMD(0X8D); // 设置DCDC命令
	OLED_WR_CMD(0X10); // DCDC关闭
	OLED_WR_CMD(0XAE); // 显示关闭
}

// 设置OLED显示位置
// x: 列坐标(0~127)
// y: 页坐标(0~7)
void OLED_Set_Pos(uint8_t x, uint8_t y)
{
	OLED_WR_CMD(0xb0 + y);				   // 设置页地址
	OLED_WR_CMD(((x & 0xf0) >> 4) | 0x10); // 设置列地址高4位
	OLED_WR_CMD(x & 0x0f);				   // 设置列地址低4位
}

// OLED全屏点亮
void OLED_On(void)
{
	uint8_t i, n;
	for (i = 0; i < 8; i++)
	{
		OLED_WR_CMD(0xb0 + i); // 设置页地址，0~7页
		OLED_WR_CMD(0x00);	   // 设置显示位置低地址
		OLED_WR_CMD(0x10);	   // 设置显示位置高地址
		for (n = 0; n < 128; n++)
			OLED_WR_DATA(1); // 写入1，点亮所有像素
	}
}

// 幂运算函数
// m: 底数
// n: 指数
unsigned int oled_pow(uint8_t m, uint8_t n)
{
	unsigned int result = 1;
	while (n--)
		result *= m;
	return result;
}

// 显示十进制数字
// x,y : 起始坐标
// len : 数字的位数
// size2: 字体大小
// mode: 显示模式  0,填充模式;1,叠加模式
// num: 数值(0~4294967295)
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2)
{
	{
		uint8_t t, temp;
		uint8_t enshow = 0;
		uint8_t char_w = size2 / 2;

		for (t = 0; t < len; t++)
		{
			// 动态计算当前的位值，避免重复调用高开销函数
			uint32_t divisor = OLED_Pow(10, len - t - 1);
			temp = (num / divisor) % 10;

			if (enshow == 0 && t < (len - 1))
			{
				if (temp == 0)
				{
					OLED_ShowChar(x + char_w * t, y, ' ', size2);
					continue;
				}
				else
					enshow = 1;
			}
			OLED_ShowChar(x + char_w * t, y, temp + '0', size2);
		}
	}
}

/**
 * @brief  显示带符号的整数
 * @param  x, y: 起始坐标
 * @param  num: 要显示的数字 (-32768 ~ 32767)
 * @param  len: 数字部分的长度（不含负号位）
 * @param  size2: 字体大小
 */
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int num, uint8_t len, uint8_t size2)
{
	uint32_t abs_num;
	uint8_t char_size = size2 / 2; // 字符宽度

	if (num < 0)
	{
		OLED_ShowChar(x, y, '-', size2); // 显示负号
		abs_num = -num;
	}
	else
	{
		OLED_ShowChar(x, y, ' ', size2); // 正数显示空格，用于覆盖之前的负号
		abs_num = num;
	}

	// 在符号位之后显示具体的数字块
	OLED_ShowNum(x + char_size, y, abs_num, len, size2);
}

/**
 * @brief  简易显示两位小数的浮点数 (如 12.34)
 * @param  num: 浮点数
 */
void OLED_Display_Angle_Fast(float num)
{
	int full_val = (int)(num * 100);		// 放大100倍，12.3456 -> 1234
	int integer_part = full_val / 100;		// 得到 12
	int decimal_part = abs(full_val) % 100; // 得到 34

	// 1. 显示带符号的整数部分 (假设整数部分最多3位)
	// 坐标 (0,0)
	OLED_ShowSignedNum(0, 0, integer_part, 3, 16);

	// 2. 在整数部分后面点个点
	// 16号字体宽度是8，符号1位+整数3位 = 4位 -> 4*8 = 32像素
	OLED_ShowChar(32, 0, '.', 16);

	// 3. 显示小数部分 (固定2位长度，不用符号)
	OLED_ShowNum(40, 0, decimal_part, 2, 16);
}

/**
 * @brief  显示带符号的浮点数 (例如: -12.34 或  1.20)
 * @param  x, y: 起始坐标
 * @param  num: 要显示的浮点数
 * @param  int_len: 整数部分的显示位数 (不含正负号)
 * @param  dec_len: 小数部分的显示位数 (建议 2)
 * @param  size: 字体大小 (12/16)
 */
void OLED_ShowSignedFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t size)
{
    uint8_t char_w = size / 2;
    uint32_t factor = OLED_Pow(10, dec_len);
    
    // 1. 处理正负号 (提前判断，解决 -0.xxx 的问题)
    if (num < 0)
    {
        OLED_ShowChar(x, y, '-', size);
        num = -num;
    }
    else
    {
        OLED_ShowChar(x, y, ' ', size);
    }

    // 2. 四舍五入处理
    // 核心逻辑：将浮点数放大后加0.5再转整数，统一处理进位
    // 例如 1.999 -> 199.9 -> 200.4 -> 200
    uint32_t total_val = (uint32_t)(num * factor + 0.5f);
    
    uint32_t integer_part = total_val / factor;
    uint32_t decimal_part = total_val % factor;

    // 3. 显示整数部分
    OLED_ShowNum(x + char_w, y, integer_part, int_len, size);

    // 4. 显示小数点
    OLED_ShowChar(x + char_w * (int_len + 1), y, '.', size);

    // 5. 显示小数部分 (强制显示前导0，例如 0.05 的 0)
    // 注意：小数部分不能用 OLED_ShowNum 的跳过前置0逻辑
    for(uint8_t i = 0; i < dec_len; i++)
    {
        uint32_t dec_divisor = OLED_Pow(10, dec_len - i - 1);
        uint8_t display_digit = (decimal_part / dec_divisor) % 10;
        OLED_ShowChar(x + char_w * (int_len + 2 + i), y, display_digit + '0', size);
    }
}


// 在指定位置显示一个字符（包括ASCII字符）
// x: 列坐标(0~127)
// y: 页坐标(0~63)
// chr: 要显示的字符
// Char_Size: 字体大小 16(8x16)/8(6x8)
// mode: 显示模式  0,反白显示;1,正常显示
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size)
{
	unsigned char c = 0, i = 0;
	c = chr - ' '; // 获取偏移后的值
	if (x > 128 - 1)
	{
		x = 0;
		y = y + 2;
	}					 // 超出屏幕则换行
	if (Char_Size == 16) // 8x16字体
	{
		OLED_Set_Pos(x, y);
		for (i = 0; i < 8; i++)
			OLED_WR_DATA(F8X16[c * 16 + i]);
		OLED_Set_Pos(x, y + 1);
		for (i = 0; i < 8; i++)
			OLED_WR_DATA(F8X16[c * 16 + i + 8]);
	}
	else // 6x8字体
	{
		OLED_Set_Pos(x, y);
		for (i = 0; i < 6; i++)
			OLED_WR_DATA(F6x8[c][i]);
	}
}

// 显示一个字符串
// x,y: 起始坐标
// chr: 字符串指针
// Char_Size: 字体大小
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t Char_Size)
{
    uint8_t j = 0;
    uint8_t char_w = (Char_Size == 16) ? 8 : 6; // 自动识别字符宽度
    
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j], Char_Size);
        x += char_w;
        if(x > (128 - char_w)) // 自动换行
        {
            x = 0; 
            y += (Char_Size == 16) ? 2 : 1; 
        }
        j++;
    }
}

// 显示汉字
// x,y: 起始坐标
// no: 汉字在字库中的索引
// hzk: 取模生成的汉字库数组
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
	uint8_t t, adder = 0;
	OLED_Set_Pos(x, y);
	for (t = 0; t < 16; t++)
	{
		OLED_WR_DATA(Hzk[2 * no][t]);
		adder += 1;
	}
	OLED_Set_Pos(x, y + 1);
	for (t = 0; t < 16; t++)
	{
		OLED_WR_DATA(Hzk[2 * no + 1][t]);
		adder += 1;
	}
}
