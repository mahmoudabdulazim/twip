class cPlaneEKF : public Kalman::EKFilter<double,1> 
{
	public:
        	cPlaneEKF();

	protected:

        	void makeA();
	        void makeH();
	        void makeV();
	        void makeR();
	        void makeW();
	        void makeQ();
	        void makeProcess();
	        void makeMeasure();
};
