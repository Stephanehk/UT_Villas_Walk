#include "walk/ActionCommand.hpp"

void ActionCommand::make_from_walk_command(walk_msg::msg::Walk::SharedPtr walk_command)
{
    forward = walk_command->forward;
    left = walk_command->left;
    turn = walk_command->turn;
    power = walk_command->power;
    bend = walk_command->bend;
    speed = walk_command->speed;
    blocking = false;
    //blocking =walk_command->blocking;
}