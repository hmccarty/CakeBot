#ifndef _PID_H_
#define _PID_H_

class PID {
    public:
        PID(double *actual, double *goal,
            double kp, double ki, double kd,
            double max_output, double min_output);

        PID(double *actual, double *goal,
            double kp, double ki, double kd,
            double max_output, double min_output,
            unsigned long sample_time);

        double calculate(unsigned long curr_time);
        void execute();

        double get_kp();
        double get_ki();
        double get_kd();

        double get_max();
        double get_min();

        unsigned long get_sample_time();

        void set_p(double kp);
        void set_i(double ki);
        void set_d(double kd);

        void set_max(double max_output);
        void set_min(double min_output);

        void set_sample_time(unsigned long sample_time);

        void set_output(double *output);

    private:
        double *actual;
        double *goal;
        double *output;
        
        double kp;
        double ki;
        double kd;

        double max_output;
        double min_output;

        double prev_actual;
        double i_sum;

        unsigned long sample_time;
        unsigned long prev_time;
};

#endif
