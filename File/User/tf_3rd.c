#include "tf_3rd.h"

void TF_3RD_init(TF_3RD_h h, float num_0, float num_1, float num_2, float num_3, float den_0, float den_1, float den_2, float den_3)
{
    h->num[0] = num_0;
    h->num[1] = num_1;
    h->num[2] = num_2;
    h->num[3] = num_3;
    h->den[0] = den_0;
    h->den[1] = den_1;
    h->den[2] = den_2;
    h->den[3] = den_3;
}


float TF_3RD_step(TF_3RD_h h, float dt, float r)
{
    h->r = r;
    h->u = h->num[0] * h->r + h->num[1]*h->r_pre + h->num[2]*h->r_pre2 + h->num[3]*h->r_pre3
        - h->den[1] * h->u_pre - h->den[2] * h->u_pre2 - h->den[3] * h->u_pre3;
    h->r_pre3 = h->r_pre2;
    h->r_pre2 = h->r_pre;
    h->r_pre = h->r;
    h->u_pre3 = h->u_pre2;
    h->u_pre2 = h->u_pre;
    h->u_pre = h->u;
    return h->u;
}


float TF_3RD_get_out(TF_3RD_h h)
{
    return h->u;
}