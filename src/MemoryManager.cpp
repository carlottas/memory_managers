//
// Created by carlotta on 26/06/17.
//

#include "MemoryManager.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include "pitt_msgs/TrackedShapes.h"
#include "pitt_msgs/TrackedShape.h"
#include <ros/console.h>
#include "memory_library/memoryManager.h"
#include "sit_msgs/EpisodicInterface.h"
#include "sit_msgs/SemanticInterface.h"
#include "sit_msgs/ScoreInterface.h"
#include <vector>
using namespace pitt_msgs;
using namespace ros;
using namespace sensor_msgs;
using namespace std;
using namespace memManager;
using namespace sit_msgs;

TrackedShapes correctedRansacShapes;
bool receivedNewShapes=false;
bool processPittInfo= false;

vector<float> coefficientsFromRansacToSemantic(TrackedShape shape);

void ransac_shape_acquisition(TrackedShapes RansacShapes){
    if(processPittInfo) {

        for (int i = 0; i < RansacShapes.tracked_shapes.size(); i++) {
            TrackedShape shape = RansacShapes.tracked_shapes[i];
            cout << shape.object_id << endl;
            cout << shape.shape_tag << endl;
            cout << "x centroid " << shape.x_est_centroid << endl;
            cout << "y centroid " << shape.y_est_centroid << endl;
            cout << "z centroid " << shape.z_est_centroid << endl;

            cout << "coefficients" << endl;
            for (int j = 0; j < shape.coefficients.size(); j++) {
                cout << shape.coefficients[j] << endl;
            }
            cout << "color " << shape.color.data << endl;

        }
        if(userCheck()){
            correctedRansacShapes=RansacShapes;
        }
        else{
            cout << "correct the shape" << endl;
            for (int i = 0; i < RansacShapes.tracked_shapes.size(); i++) {
                TrackedShape shape = RansacShapes.tracked_shapes[i];
                TrackedShape shapeNew;
                cout << "shapeId" << endl;
                cout << shape.object_id << endl;
                cout << "shapeTag" << endl;
                cout << shape.shape_tag << endl;
                cout << "type new shape tag" << endl;
                string newShapeTag;
                getline(cin, newShapeTag);
                if (newShapeTag == SPHERE ||
                    newShapeTag == CONE ||
                    newShapeTag == CYLINDER ||
                    newShapeTag == PLANE) {
                    shapeNew.object_id = shape.object_id;
                    shapeNew.shape_tag = newShapeTag;
                    if (userCheckValues("centroids information")) {
                        float newXCentroid;
                        cout << "x centroid " << shape.x_est_centroid << endl;
                        cout << "insert new x centroid" << endl;
                        cin >> newXCentroid;
                        shapeNew.x_est_centroid = newXCentroid;
                        float newYCentroid;
                        cout << "y centroid " << shape.y_est_centroid << endl;
                        cout << "insert new y centroid" << endl;
                        cin >> newYCentroid;
                        shapeNew.y_est_centroid = newYCentroid;
                        float newZCentroid;
                        cout << "z centroid " << shape.z_est_centroid << endl;
                        cout << "insert new z centroid" << endl;
                        cin >> newZCentroid;
                        shapeNew.z_est_centroid = newYCentroid;
                    } else {
                        shapeNew.x_est_centroid = shape.x_est_centroid;
                        shapeNew.y_est_centroid = shape.y_est_centroid;
                        shapeNew.z_est_centroid = shape.z_est_centroid;
                    }
                    if (newShapeTag == shape.shape_tag) {
                        if (userCheckValues("coefficients information")) {
                            vector<float> coefficientNew;
                            for (int j = 0; j < shape.coefficients.size(); j++) {
                                float newCoefficient;
                                cout << shape.coefficients[j] << endl;
                                cout << "insert new coefficinet" << endl;
                                cin >> newCoefficient;
                                coefficientNew.push_back(newCoefficient);
                            }
                            shapeNew.coefficients = coefficientNew;
                        } else {
                            shapeNew.coefficients = shape.coefficients;
                        }
                    } else {
                        vector<float> coefficientNew;
                        cout << "insert new coefficients" << endl;
                        bool addOtherElements;
                        do {
                            float coeff;
                            cout << "insert coefficient" << endl;
                            cin >> coeff;
                            coefficientNew.push_back(coeff);
                            addOtherElements = userCheckContinue("adding coefficients");
                        } while (addOtherElements);
                        shapeNew.coefficients = coefficientNew;


                    }

                    if (userCheckValues("color")) {
                        string newColor;
                        cout << "color " << shape.color.data << endl;
                        cout << "insert new color " << endl;
                        getline(cin, newColor);
                        shapeNew.color.data = newColor;
                    } else {
                        shapeNew.color.data = shape.color.data;
                    }
                    correctedRansacShapes.tracked_shapes.push_back(shapeNew);

                }


            }
            receivedNewShapes = true;
        }
        processPittInfo=false;
    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, MEMORIZATION_MANAGER_NODE_NAME);
    ros::NodeHandle node;
    ros::NodeHandle* nodePtr=&node;
    // set subscriber to get perceived shape informations
    ros::Subscriber subPITT = node.subscribe(PITT_TOPIC,1000, ransac_shape_acquisition);
    //declaration of the services
    ServiceClient client_semantic = nodePtr->serviceClient < SemanticInterface > (SRV_SEMANTIC);
    SemanticInterface srv_semantic;
    ServiceClient client_episodic = nodePtr->serviceClient < EpisodicInterface > (SRV_EPISODIC);
    EpisodicInterface srv_episodic;
    ServiceClient client_score = nodePtr->serviceClient < ScoreInterface > (SRV_SCORE);
    ScoreInterface srv_score;
    processPittInfo= true;
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if(receivedNewShapes){
            //Calling the scene service
            cout<<"received the atoms information"<<endl;
            //calling the semantic service
            for (int i=0; i<correctedRansacShapes.tracked_shapes.size(); i++){
                cout<<"filling the srv request "<<endl;
                atom shapeSemanticService;
                TrackedShape shapeRansac = correctedRansacShapes.tracked_shapes[i];
                shapeSemanticService.type=shapeRansac.shape_tag;
                shapeSemanticService.coefficients=coefficientsFromRansacToSemantic(shapeRansac);
                shapeSemanticService.color=shapeRansac.color.data;
                srv_semantic.request.geometricPrimitives.push_back(shapeSemanticService);
            }
            cout<<"calling the service"<<endl;
            if(client_semantic.call(srv_semantic)){
                cout<<"called the service"<<endl;
                //calling the episodic service
                srv_episodic.request.SceneName=srv_semantic.response.SceneName;
                srv_episodic.request.SubClasses=srv_semantic.response.SubClasses;
                srv_episodic.request.SuperClasses=srv_semantic.response.SuperClasses;
                srv_episodic.request.Object=srv_semantic.response.Objects;
                string support="";
                cout<<"please insert the support Name"<<endl;
                getline(cin, support);
                srv_episodic.request.SupportName=support;

                if(client_episodic.call(srv_episodic)){
                    if(srv_semantic.response.learnt){
                        SemanticScoreItem semanticScoreItem;
                        semanticScoreItem.Name= srv_semantic.response.SceneName;
                        semanticScoreItem.FirstSuperClass=srv_semantic.response.FirstSuperClass;
                        semanticScoreItem.SubClasses=srv_semantic.response.SubClasses;
                        semanticScoreItem.SuperClasses=srv_semantic.response.SuperClasses;
                        //TODO change because it has to be two string instead now it is a string and a List
                        semanticScoreItem.IsFirstSuperCLassOf=srv_semantic.response.isFirstSuperClassOf;
                        srv_score.request.Semantic=semanticScoreItem;
                    }
                    EpisodicScoreItem episodicScoreItem;
                    episodicScoreItem.Name= srv_episodic.response.EpisodicSceneName;
                    episodicScoreItem.NameSemanticItem=srv_semantic.response.SceneName;
                    srv_score.request.Episodic=episodicScoreItem;
                    if(client_score.call(srv_score)){
                        }

                }


            }

        }
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}

vector <float> coefficientsFromRansacToSemantic(TrackedShape shape){
    vector <float> coefficientsRansac;
    //todo change in the semantic srv the order in which the coefficients are given
    coefficientsRansac.push_back(shape.x_est_centroid);
    coefficientsRansac.push_back(shape.y_est_centroid);
    coefficientsRansac.push_back(shape.z_est_centroid);
    coefficientsRansac.insert(coefficientsRansac.end(),shape.coefficients.begin(),shape.coefficients.end());
    return coefficientsRansac;


}

