#ifndef JRC18SIA_TARGET_INFO_H
#define JRC18SIA_TARGET_INFO_H

#include <string>

#include <geometry_msgs/Pose.h>

// Desk height
#define DESK_HEIGHT_LOW 0.3
#define DESK_HEIGHT_MIDDLE 0.6
#define DESK_HEIGHT_HIGH 0.8

enum GraspType
{
    MOVE,
    SUCK,
    GRASP
};

struct Header
{
    std::string target_name;
    int target_id;
    GraspType grasp_type;

    Header() {}
};

struct ShapeInfo
{
    double length;
    double width;
    double height;
    double center;
    ShapeInfo() {}
};

struct GraspInfo
{
    double center; // grasp position
    double height;
    double up;
    double down;

    GraspInfo() {}
};

struct DeskHeightInfo
{
    DeskHeightInfo()
        : low(DESK_HEIGHT_LOW), middle(DESK_HEIGHT_MIDDLE), high(DESK_HEIGHT_HIGH)
    {}
    double low;
    double middle;
    double high;
};

struct TargetInfo
{
    Header header;
    ShapeInfo shape_info;
    GraspInfo grasp_info;
    TargetInfo(){}
};

class JRCTarget
{
public:
    JRCTarget()
    {
        target0_book.header.target_name = "book"; // 书
        target0_book.header.target_id = 0;
        target0_book.header.grasp_type = SUCK;
        target0_book.shape_info.length = 0.1;
        target0_book.shape_info.width = 0.1;
        target0_book.shape_info.height = 0.1;
        target0_book.grasp_info.up = 0.1;

        target1_toothbrush.header.target_name = "toothbrush"; // 牙刷
        target1_toothbrush.header.target_id = 1;
        target1_toothbrush.header.grasp_type = SUCK; // or MOVE
        target1_toothbrush.grasp_info.up = 0.1;

        target2_can.header.target_name = "can"; // 可乐
        target2_can.header.target_id = 2;
        target2_can.header.grasp_type = MOVE;
        target2_can.grasp_info.up = 0.1;

        target3_strips.header.target_name = "strips"; // 薯条 桶装
        target3_strips.header.target_id = 3;
        target3_strips.header.grasp_type = SUCK;
        target3_strips.grasp_info.up = 0.1;

        target4_chips.header.target_name = "chips"; // 薯片 袋装
        target4_chips.header.target_id = 4;
        target4_chips.header.grasp_type = SUCK;
        target4_chips.grasp_info.up = 0.1;

        target5_oreo.header.target_name = "oreo"; // 奥利奥
        target5_oreo.header.target_id = 5;
        target5_oreo.header.grasp_type = SUCK;
        target5_oreo.grasp_info.up = 0.1;

        target6_pacific.header.target_name = "pacific"; // 太平苏打饼干
        target6_pacific.header.target_id = 6;
        target6_pacific.header.grasp_type = SUCK;
        target6_pacific.grasp_info.up = 0.1;

        target7_shampoo.header.target_name = "shampoo"; // 洗发精
        target7_shampoo.header.target_id = 7;
        target7_shampoo.header.grasp_type = MOVE;
        target7_shampoo.grasp_info.up = 0.1;

        target8_tissue.header.target_name = "tissue"; // 纸巾
        target8_tissue.header.target_id = 8;
        target8_tissue.header.grasp_type = SUCK;
        target8_tissue.grasp_info.up = 0.1;

        target9_sausage.header.target_name = "sausage"; // 香肠
        target9_sausage.header.target_id = 9;
        target9_sausage.header.grasp_type = SUCK;
        target9_sausage.grasp_info.up = 0.1;

        target10_toothpaste.header.target_name = "toothpaste"; // 牙膏
        target10_toothpaste.header.target_id = 10;
        target10_toothpaste.header.grasp_type = SUCK; // or MOVE
        target10_toothpaste.grasp_info.up = 0.1;

        target11_teether.header.target_name = "teether"; // 磨牙棒（婴孩长牙时咬的玩具）
        target11_teether.header.target_id = 11;
        target11_teether.header.grasp_type = SUCK; // or MOVE
        target11_teether.grasp_info.up = 0.1;

        target12_milk.header.target_name = "milk"; // 牛奶
        target12_milk.header.target_id = 12;
        target12_milk.header.grasp_type = MOVE;
        target12_milk.grasp_info.up = 0.1;

        target13_jelly.header.target_name = "jelly"; // 果冻
        target13_jelly.header.target_id = 13;
        target13_jelly.header.grasp_type = MOVE;
        target13_jelly.grasp_info.up = 0.1;

        target0_book.header.target_name = "book";
        target0_book.header.target_id = 14;
        target0_book.header.grasp_type = SUCK;

        target0_book.header.target_name = "book";
        target0_book.header.target_id = 15;
        target0_book.header.grasp_type = SUCK;

        target0_book.header.target_name = "book";
        target0_book.header.target_id = 16;
        target0_book.header.grasp_type = SUCK;

        target0_book.header.target_name = "book";
        target0_book.header.target_id = 17;
        target0_book.header.grasp_type = SUCK;

        target0_book.header.target_name = "book";
        target0_book.header.target_id = 18;
        target0_book.header.grasp_type = SUCK;

        target0_book.header.target_name = "book";
        target0_book.header.target_id = 19;
        target0_book.header.grasp_type = SUCK;

        target0_book.header.target_name = "book";
        target0_book.header.target_id = 20;
        target0_book.header.grasp_type = SUCK;

    }

    TargetInfo target0_book; // 书
    TargetInfo target1_toothbrush; // 牙刷
    TargetInfo target2_can; // 可乐
    TargetInfo target3_strips; // 薯条 桶装
    TargetInfo target4_chips; // 薯片 袋装
    TargetInfo target5_oreo; // 奥利奥
    TargetInfo target6_pacific; // 太平苏打饼干
    TargetInfo target7_shampoo; // 洗发精
    TargetInfo target8_tissue; // 纸巾
    TargetInfo target9_sausage; // 香肠
    TargetInfo target10_toothpaste; // 牙膏
    TargetInfo target11_teether; // 磨牙棒（婴孩长牙时咬的玩具）
    TargetInfo target12_milk; // 牛奶
    TargetInfo target13_jelly; // 果冻
    TargetInfo target14_;
    TargetInfo target15_;
    TargetInfo target16_;
    TargetInfo target17_;
    TargetInfo target18_;
    TargetInfo target19_;
    TargetInfo target20_;

};


#endif // JRC18SIA_TARGET_INFO_H
