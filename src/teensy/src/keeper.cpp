#include "keeper.h"
#include <math.h>

Vec2 keeper(Vec2 Player, Vec2 ball){
    Vec2 Torkordinate = {0,-120};
    
    Vec2 tp_diff = Torkordinate - Player;
    Vec2 bp_diff = ball - Player;
    
    Vec2 test = (tp_diff + bp_diff)/2;

    return test;
}