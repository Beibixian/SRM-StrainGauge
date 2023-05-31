//*******************************************************
// 一次元フーリエ変換プログラム *
// 1.変換用データを作成 *
// ２.変換を行い,複素数の変換結果を出力 *
// ３.絶対値を取り,振幅スペクトルを出力 *
//*******************************************************
#include <stdio.h>
#include <math.h>
#include <complex.h>
#define PI 3.14159265358
#define N_fft 1024 // データ個数
void main()
{
    int n_fft, k_fft;
    double R[N_fft], max, phase[N_fft], magnitude[N_fft];
    double complex Z;
    typedef struct
    {
        double r;
        double i;
    } complex_define;
    complex_define x[N_fft], X;

    // 実験用入力データを用意する.
    for (n_fft = 0; n_fft < N_fft; n_fft++)
    {
        x[n_fft].r = 1 + sin(2 * PI * 1 * n_fft / N_fft);
        x[n_fft].i = 0.0;
    }
    for (n_fft = 0; n_fft < N_fft; n_fft++)
    {
        printf("sin[%d]=%3.3lf\n", n_fft, x[n_fft]);
    }
    max = 0.0;

    // ここからはフーリエ変換の中心部です.
    for (n_fft = 0; n_fft < N_fft; n_fft++)
    {
        X.r = 0;
        X.i = 0;
        for (k_fft = 0; k_fft < N_fft; k_fft++)
        {
            X.r += x[k_fft].r * cos(2.0 * PI * n_fft * k_fft / N_fft) + x[k_fft].i * sin(2.0 * PI * n_fft * k_fft / N_fft);
            X.i += x[k_fft].i * cos(2.0 * PI * n_fft * k_fft / N_fft) - x[k_fft].r * sin(2.0 * PI * n_fft * k_fft / N_fft);
        }
        R[n_fft] = sqrt(X.r * X.r + X.i * X.r);
        Z = X.r + I * X.i;

        phase[n_fft] = carg(Z);     // 计算相位
        magnitude[n_fft] = cabs(Z); // 计算幅度
    }
}