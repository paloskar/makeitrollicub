#include <string>
#include <stdio.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/***************************************************/
class CtrlModule: public RFModule
{
protected:
	yarp::sig::Vector ang;
	PolyDriver *clientGazeCtrl;
	PolyDriver *clientArmCtrl;
	PolyDriver *robotDevice;
	IPositionControl *pos;
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;
	Vector x0,o0;
	int startup_context_id;
	BufferedPort<ImageOf<PixelRgb>> imgLPortIn,imgRPortIn;
    Port imgLPortOut,imgRPortOut;
    RpcServer rpcPort;
    
    Vector cogL,cogR;
    bool okL,okR;

    /***************************************************/
    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    {
        int xMean=0;
        int yMean=0;
        int ct=0;

        for (int x=0; x<img.width(); x++)
        {
            for (int y=0; y<img.height(); y++)
            {
                PixelRgb &pixel=img.pixel(x,y);
                if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g))
                {
                    xMean+=x;
                    yMean+=y;
                    ct++;
                }
            }
        }

        if (ct>0)
        {
            cog.resize(2);
            cog[0]=xMean/ct;
            cog[1]=yMean/ct;
            return true;
        }
        else
            return false;
    }

    /***************************************************/
    Vector retrieveTarget3D(const Vector &ctgL, const Vector &ctgR)
    {
		Vector x;
		bool ret = igaze->triangulate3DPoint(ctgL,ctgR,x);
        // FILL IN THE CODE;
		return x;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
		igaze->lookAtFixationPoint(x);
        // FILL IN THE CODE
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
		Matrix R(3,3);
		R(0,0)=-1.0; R(0,1)= 0.0; R(0,2)= 0.0; // x-coordinate
		R(1,0)= 0.0; R(1,1)= 0.0; R(1,2)=-1.0; // y-coordinate
		R(2,0)=0.0; R(2,1)= -1.0; R(2,2)= 0.0; // z-coordinate
		Vector o=dcm2axis(R);
		return o;
        // FILL IN THE CODE
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o)
    {
		iarm->setTrackingMode(true);
		iarm->setTrajTime(0.5);
		iarm->setPosePriority("orientation");
		iarm->goToPose(x,o);
		iarm->waitMotionDone();
        // FILL IN THE CODE
    }

    /***************************************************/
    void makeItRoll(const Vector &x, const Vector &o)
    {
		// FILL IN THE CODE
		iarm->setTrackingMode(true);
		iarm->setTrajTime(0.5);
		iarm->setPosePriority("orientation");
		iarm->goToPose(x,o);
		iarm->waitMotionDone();
    }

    /***************************************************/
    void look_down()
    {
		pos->positionMove(0,-40.0);
        // FILL IN THE CODE
    }

    /***************************************************/
    void roll(const Vector &cogL, const Vector &cogR)
    {
        printf("detected cogs = (%s) (%s)\n",
                cogL.toString(0,0).c_str(),cogR.toString(0,0).c_str());

        Vector x=retrieveTarget3D(cogL,cogR);
        printf("retrieved 3D point = (%s)\n",x.toString(3,3).c_str());

        fixate(x);
        printf("fixating at (%s)\n",x.toString(3,3).c_str());

        Vector o=computeHandOrientation();
        printf("computed orientation = (%s)\n",o.toString(3,3).c_str());

		x[1]=x[1]+0.1;		// adjusting to place hand right in front of ball
		x[2]=x[2]+0.03;
        approachTargetWithHand(x,o);
        printf("approached\n");

		x[1]=x[1]-0.1;		// move hand above desk to roll ball 
        makeItRoll(x,o);
        printf("roll!\n");
    }

    /***************************************************/
    void home()
    {
		igaze->lookAtAbsAngles(ang);
		iarm->goToPose(x0,o0);
        // FILL IN THE CODE
    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
		okL=false;
		okR=false;
		Property options;
		printf("Configure.\n");
		options.put("device", "remote_controlboard");
		options.put("local", "/neckclient");                 //local port names
		options.put("remote", "/icubSim/head");         //where we connect to

		Property optArm;
		optArm.put("device","cartesiancontrollerclient");
		optArm.put("remote","/icubSim/cartesianController/right_arm");
		optArm.put("local","/client/right_arm");
		clientArmCtrl = new PolyDriver();
		clientArmCtrl->open(optArm);
		clientArmCtrl->view(iarm);

		Property optGaze;
		optGaze.put("device", "gazecontrollerclient");
		optGaze.put("remote", "/iKinGazeCtrl");
		optGaze.put("local", "/make-it-roll");

		clientGazeCtrl = new PolyDriver();
		clientGazeCtrl->open(optGaze);

		clientGazeCtrl->view(igaze);
		ang.resize(3);
		igaze->getAngles(ang);

		robotDevice = new PolyDriver();
		robotDevice->open(options);

		if (!robotDevice->isValid()) {
			printf("Device not available.  Here are the known devices:\n");
			printf("%s", Drivers::factory().toString().c_str());
			return 1;
		} 

		robotDevice->view(pos);
		if (pos == 0) {
			printf("Error getting IPositionControl interface.\n");
			return 1;
		}
		else
		{
			printf("\npos control works\n");
		}

        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");

        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

        rpcPort.open("/service");
        attach(rpcPort);
		iarm->getPose(x0,o0);
		printf("\nConfiguring done!\n");

		Vector curDof;
		iarm->getDOF(curDof); // [0 0 0 1 1 1 1 1 1 1]
		Vector newDof(3);
		newDof[0]=1; // torso pitch: 1 => enable
		newDof[1]=2; // torso roll: 2 => skip
		newDof[2]=1; // torso yaw: 1 => enable
		iarm->setDOF(newDof,curDof);

		return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
		printf("\n\nClosing!\n\n");
        delete clientGazeCtrl;
		delete robotDevice;
		printf("Closing again\n");
		drvArm.close();
        drvGaze.close();
        imgLPortIn.close();
        imgRPortIn.close();
        imgLPortOut.close();
        imgRPortOut.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString().c_str();
        if (cmd=="look_down")
        {
            look_down();
            reply.addString("Yep! I'm looking down now!");
            return true;
        }
        else if (cmd=="roll")
        {
            // FILL IN THE CODE
			if (okL && okR)
            {
                roll(cogL,cogR);
                reply.addString("Yeah! I've made it roll like a charm!");
            }
            else
                reply.addString("I don't see any object!");

            return true;
        }
        else if (cmd=="home")
        {
            home();
            reply.addString("I've got the hard work done! Going home.");
            return true;
        }
        else
            return RFModule::respond(command,reply);
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images
    }

    /***************************************************/
    bool updateModule()
    {



        // get fresh images
//		printf("Blah!");
//		pos->positionMove(1,0.0);
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();
//        ang[0]=0.0;
//        ang[1]=-120.0;
//        ang[2]=0.0;
//        igaze->lookAtAbsAngles(ang);

//		printf("Updating.\n");
        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL))
            return false;

        // compute the center-of-mass of pixels of our color
//        mutex.lock();
        okL=getCOG(*imgL,cogL);
        okR=getCOG(*imgR,cogR);
//        mutex.unlock();

        PixelRgb color;
        color.r=255; color.g=0; color.b=0;

        if (okL)
            draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);
        color.r=0; color.g=255; color.b=0;
        if (okR)
            draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);

        imgLPortOut.write(*imgL);
        imgRPortOut.write(*imgR);

        return true; 
    }
};


/***************************************************/
int main()
{   
	YARP_REGISTER_DEVICES(icubmod)
	Network yarp;
	printf("Start.\n");
    if (!yarp.checkNetwork())
        return -1;
    CtrlModule mod;
    ResourceFinder rf;
	printf("Yarp ok.\n");
	return mod.runModule(rf);
	printf("End.\n");
}



