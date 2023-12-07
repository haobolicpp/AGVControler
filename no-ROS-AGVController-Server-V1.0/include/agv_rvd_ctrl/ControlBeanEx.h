#ifndef CONTROLBEANEX_H
#define CONTROLBEANEX_H

class ControlBeanEx {
public:
    float x, y, z, goal_angle1, goal_angle2, rotation;
    bool communicate_success, initial_finish, servo_off_flag, move_flag;
    float angle1_after_judge, angle2_after_judge;
    bool isReach_after_judge;
    float real_x, real_y, real_z, real_angle1, real_angle2, real_rotation;
    int tool_hand;
    float Tool_L;
    float Tool_R;
    float base_x, base_y, base_r;

    ControlBeanEx();
    ~ControlBeanEx();
    bool    is_connected();
    int     initial(int generation, float z_travel);
    int     set_angle_move(float angle1, float angle2, float z, float rotation,float speed);
    bool    judge_in_range(float x, float y, float z, float ratation);
    void    get_scara_param();
    bool    is_collision();
    int     get_card_num();
    void    set_arm_length(float l1, float l2);
    int     change_attitude(float speed);
    int     single_axis_move(int axis, float distance);
    int     trail_move(int point_number, float* x, float* y, float* z, float* r, float speed);
    int     set_position_move(float goal_x, float goal_y, float goal_z, float rotation, float speed, float acceleration,int interpolation, int move_mode);
    int     xyz_move(int direction, float distance, float speed);
    void    stop_move();
    bool    set_digital_out(int io_number, bool value);
    int     unlock_position();
    int     get_digital_in(int io_in_number);
    int     set_efg_state(int type, float distance);
    int     get_efg_state(int type, float* distance);
    int     get_efg_state(int* type, float* distance);
    int     get_efg_state_dji(int* type, float* distance);
    int     set_efg_state_dji(int type, float distance);
    int     get_digital_out(int io_out_num);
    bool    set_cooperation_fun_state(bool state);
    bool    get_cooperation_fun_state();
    bool    set_drag_teach(bool state);
    bool    get_drag_teach();
    bool    judge_position_gesture(float x, float y);
    void    get_robot_real_coor();
    bool    is_robot_goto_target();
    int     single_joint_move(int axis, float distance, float speed);
    void    set_allow_distance_at_target_position(float x_distance, float y_distance, float z_distance, float r_distance);
    int     start_joint_monitor();
    int     get_joint_state(int joint_num);
    int     joint_home(int joint_num);
    void    set_catch_or_release_accuracy(float accuracy);
    int     movel_xyz(float goal_x, float goal_y, float goal_z, float rotation, float speed);
    int     movej_xyz(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float rough);
    int     movej_angle(float angle1, float angle2, float goal_z, float goal_r, float speed, float rough);
    bool    wait_stop();
    void    clear_move_list_buf();
    float   get_arm1_length();
    float   get_arm2_length();
    int     pause_move();
    int     resume_move();
    int     read_efrom(int addr);
    int     write_eform(int addr, int value);
    int     get_robot_id();
    int     limited_speed(bool state);
    float   get_current_comm_error(int joint_num);
    int     get_error_code();
    int     set_tool_fun1(float Tool_L,float Tool_R);
    int     set_tool_fun2(float p_x, float p_y);
    int     set_tool_fun3(float p1_x, float p1_y, float p1_r, float p2_x, float p2_y, float p2_r);
    int     new_movej_xyz_lr(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float roughly, int lr);
    int     new_movej_angle(float angle1, float angle2, float goal_z, float goal_r, float speed, float rough);
    bool    check_joint(int joint_num, bool state);
    //j5
    int		j5_motor_zero();
    int		set_j5_motor_pos(float deg, float speed);
    float	get_j5_parameter();
    int		movej_j5(float j5_pos, float speed);
    int     movel_j5_xyz(float goal_x, float goal_y, float goal_z, float goal_r, float j5, float speed);
    int     movej_j5_xyz_lr(float goal_x, float goal_y, float goal_z, float goal_r, float j5, float speed, float roughly, int lr);
    int     movej_j5_angle(float goal_angle1, float goal_angle2, float goal_z, float goal_r, float j5, float speed, float roughly);
    int     new_set_efg_state(int channel, int type, float distance);
    int     new_get_efg_state(int channel, int type, float* distance);
};


#endif  // CONTROLBEANEX_H
