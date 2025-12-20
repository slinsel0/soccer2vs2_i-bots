#include <keeper.h>
#include <math.h>

const float cam_constant = -1.923;
const int y_cam_size = 480;
const int x_cam_size = 640;


void keeper (Vec2 player,Vec2 ball){

Vec2 Torkordinate = {0,-120};


Vec2 real_ball = {ball.y * cam_constant + y_cam_size, ball.x * cam_constant + x_cam_size}; 

Vec2 pt_diff = player - Torkordinate;

Vec2 bp_diff = real_ball - player;

Vec2 tb_diff = pt_diff + bp_diff;

double t_angle = acos(tb_diff.x / sqrt( pow(tb_diff.x, 2) + pow(tb_diff.y, 2)));


}
