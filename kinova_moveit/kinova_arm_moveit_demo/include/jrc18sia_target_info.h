/*****************************
 *
 *
 * Author : Cong Wang
 * Data : 20181202
 *
 *
 *
 * ***************************/

#ifndef JRC18SIA_TARGET_INFO_H
#define JRC18SIA_TARGET_INFO_H

#include <string>

// Desk height w.r.t kinova arm base frame
#define DESK_HEIGHT_LOW (0.30 - 0.41 + 0.05)
#define DESK_HEIGHT_MIDDLE (0.60 - 0.41 + 0.05)
#define DESK_HEIGHT_HIGH (0.90 - 0.41 + 0.05)

enum GraspType
{
	SWEEP,
	SUCK
	// GRASP
};

enum TableType
{
	HIGH,
	MIDDLE,
	LOW
};

struct TargetPose
{
	double x;
	double y;
	double z;
};

struct Header
{
	std::string target_name;
	int         target_id;
	GraspType   grasp_type;
	TableType   table_type;

	Header() {}
};

struct ShapeInfo
{
	double length;
	double width;
	double height;
	ShapeInfo() {}
};

struct GraspInfo
{
	double grasp_x;
	double grasp_y;
	double grasp_z;

	GraspInfo() {}
};

struct DeskHeightInfo
{
	DeskHeightInfo() : low(DESK_HEIGHT_LOW), middle(DESK_HEIGHT_MIDDLE), high(DESK_HEIGHT_HIGH) {}
	double low;
	double middle;
	double high;
};

struct TargetInfo
{
	Header    header;
	ShapeInfo shape_info;
	GraspInfo grasp_info;
	TargetInfo() {}
};

class JRCTarget
{
  public:
	JRCTarget()
	{
		// jrc_target_info.resize(20);
		setTargetInfo(0, SUCK, 0.175, 0.228, 0.024, 0, 0, 0.1, "book");        // 书
		setTargetInfo(1, SWEEP, 0.225, 0.068, 0.018, 0, 0, 0.1, "toothbrush"); // 牙刷
		setTargetInfo(2, SWEEP, 0.063, 0.063, 0.115, 0, 0, 0.1, "can");        // 可乐
		setTargetInfo(3, SWEEP, 0.090, 0.090, 0.100, 0, 0, 0.1, "strips");     // 薯条-桶装
		setTargetInfo(4, SUCK, 0.220, 0.175, 0.060, 0, 0, 0.1, "chips");      // 薯片-袋装
		setTargetInfo(5, SWEEP, 0.130, 0.050, 0.050, 0, 0, 0.1, "oreo");       // 奥利奥
		setTargetInfo(6, SUCK, 0.160, 0.100, 0.046, 0, 0, 0.1, "pacific");     // 太平苏打饼干
		setTargetInfo(7, SWEEP, 0.050, 0.040, 0.185, 0, 0, 0.1, "shampoo");    // 洗发精
		setTargetInfo(8, SUCK, 0.152, 0.100, 0.075, 0, 0, 0.1, "tissue");      // 纸巾
		setTargetInfo(9, SWEEP, 0.123, 0.070, 0.018, 0, 0, 0.1, "sausage");    // 香肠
		setTargetInfo(10, SUCK, 0.238, 0.055, 0.043, 0, 0, 0.1, "toothpaste"); // 牙膏
		setTargetInfo(11, SWEEP, 0.140, 0.095, 0.015, 0, 0, 0.1, "teether");   // 磨牙棒
		setTargetInfo(12, SWEEP, 0.065, 0.045, 0.106, 0, 0, 0.1, "milk");      // 牛奶
		setTargetInfo(13, SWEEP, 0.080, 0.047, 0.170, 0, 0, 0.1, "jelly");     // 果冻
		setTargetInfo(14, SWEEP, 0.040, 0.040, 0.083, 0, 0, 0.1, "tablet");    // 药片
		setTargetInfo(15, SWEEP, 0.080, 0.080, 0.080, 0, 0, 0.1, "orange");    // 橘子
		setTargetInfo(16, SWEEP, 0.080, 0.080, 0.080, 0, 0, 0.1, "orange");    // TODO
		setTargetInfo(17, SWEEP, 0.080, 0.080, 0.080, 0, 0, 0.1, "orange");    // TODO
		setTargetInfo(18, SWEEP, 0.080, 0.080, 0.080, 0, 0, 0.1, "orange");    // TODO
		setTargetInfo(19, SWEEP, 0.080, 0.080, 0.080, 0, 0, 0.1, "orange");    // TODO
		std::cout << "target_info.shape_info.length: " << jrc_target_info[2].shape_info.length << std::endl;
        std::cout << "size: " << jrc_target_info.size() << std::endl;
	}

	std::vector<TargetInfo> jrc_target_info;
	void setTargetInfo(int       id,                                   // id
	                   GraspType grasp,                                // grasp type
	                   double length, double width, double height,     // shape
	                   double grasp_x, double grasp_y, double grasp_z, // grasp info
	                   std::string name)                               // name
	{
		TargetInfo target_info;
		target_info.header.target_name = name;
		target_info.header.target_id   = id;
		target_info.header.grasp_type  = grasp;
		target_info.shape_info.length  = length;
		target_info.shape_info.width   = width;
		target_info.shape_info.height  = height;
		target_info.grasp_info.grasp_x = grasp_x;
		target_info.grasp_info.grasp_y = grasp_y;
		target_info.grasp_info.grasp_z = grasp_z;

		jrc_target_info.push_back(target_info);
		std::cout << "setTargetInfo" << std::endl;
		std::cout << "target_info.shape_info.length: " << target_info.shape_info.length << std::endl;
	}
};

#endif // JRC18SIA_TARGET_INFO_H
