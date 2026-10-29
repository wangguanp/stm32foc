float low_pass_filter(float input,float last_output,float alpha)
{
    /* 全部使用 float：写成 1.0 会把表达式提升为 double，F103 上 double 软浮点非常慢 */
    return alpha * (input - last_output) + last_output;
}
/*q越大，更相信状态汇编，响应快噪声大  r越大，更相信侧脸很准：平滑强相应慢*/
/* 一维标准卡尔曼滤波（可选，用于速度等慢变量） */
float kalman_filter_std(float input, float r, float q)
{
    static float z;
    static float p = 1;
    float g = 0;
    p = p + q;  //预测：P^- = P + Q
    g = p / (p + r);  // 卡尔曼增益 K = P^- / (P^- + R)
    z = z + g * (input - z);   // 更新状态 
    p = (1 - g) * p;  // 更新协方差
    return z;
}