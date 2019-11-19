//author  Renato Sousa, 2018
#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"

#include "pb/messages_robocup_ssl_detection.pb.h"
#include "pb/messages_robocup_ssl_geometry.pb.h"
#include "pb/messages_robocup_ssl_wrapper.pb.h"
#include "pb/grSim_Packet.pb.h"
#include "pb/grSim_Commands.pb.h"
#include "pb/grSim_Replacement.pb.h"

using namespace std;

#define RADIUS // Raio de detecção
#define VNSH_LIM 5 // Limite de frames consecutivos sem detecção
#define FRM_LIM 5 // Limiar de confiança (quantidade de detecções consecutivas para promover uma bola)
#define CONFLIM // Limite inferior da confiança de uma bola (SSL_DetectionBall.confidence())
#define SUP_AREALIM // Limite superior da área de uma bola (SSL_DetectionBall.area())
#define INF_AREALIM // Limite inferior da área de uma bola (SSL_DetectionBall.area())

class myball{

    private:

    float x, y, predict_x, predict_y;
    float vx, vy;
    float tempx, tempy;
    float predict_radius;
    int framecntr;
    int vanishcntr;
    bool detected;

    public:

    myball(float x, float y){

        this.x = x;
        this.y = y;
        predict_x = x;
        predict_y = y;
        predict_radius = RADIUS*RADIUS;
        vx = 0;
        vy = 0;

        framecntr = 0;
        vanishcntr = 0;
        detected = 0;
    }
    void reset(float x, float y){

        this.x = x;
        this.y = y;
        predict_x = x;
        predict_y = y;
        predict_radius = RADIUS*RADIUS;
        vx = 0;
        vy = 0;

        framecntr = 0;
        vanishcntr = 0;
        detected = 0;

    }

    float x(){
        return predict_x;
    }
    float y(){
        return predict_y;
    }
    float vx(){
        return vx;
    }
    float vy(){
        return vy;
    }
    float predict_radius(){
        return predict_radius;
    }
    bool detected(){
        return detected;
    }
    int framecntr(){
        return framecntr;
    }
    int vanishcntr(){
        return vanishcntr;
    }
    
    void predict(){
        predict_x += vx;
        predict_y += vy;

        vanishcntr++;
        framecntr = 0;
    }

    void detection(float x, float y, float temp_r){
        tempx = x;
        tempy = y;

        predict_radius = temp_r;

        detected = 1;
    }

    void refresh_v(){
        vx = (tempx-x)/(vanishcntr+1);
        vy = (tempy-y)/(vanishcntr+1);

        x = tempx;
        y = tempy;

        predict_radius = RADIUS*RADIUS;

        predict_x = x;
        predict_y = y;

        framecntr++;
        vanishcntr = 0;
    }

};

class myrobot{

    private:

    float x, y, predict_x, predict_y;
    float vx, vy;
    int vanishcntr;
    bool detected;

    public:

    myrobot(float x, float y){

        this.x = x;
        this.y = y;
        predict_x = x;
        predict_y = y;
        vx = 0;
        vy = 0;

        vanishcntr = 0;
        detected = 0;
    }
    void undetect(){

        detected = 0;
    }

    float x(){
        return predict_x;
    }
    float y(){
        return predict_y;
    }
    bool detected(){
        return detected;
    }

    void predict(){
        predict_x += vx;
        predict_y += vy;

        vanishcntr++;
    }

    void detection(float x, float y){

        vx = (x - this.x)/(vanishcntr + 1);
        vy = (y - this.y)/(vanishcntr + 1);

        this.x = x;
        this.y = y;
        predict_x = x;
        predict_y = y;

        vanishcntr = 0;

        detected = 1;
    }
    

};

void printRobotInfo(const SSL_DetectionRobot & robot) {
    printf("CONF=%4.2f ", robot.confidence());
    if (robot.has_robot_id()) {
        printf("ID=%3d ",robot.robot_id());
    } else {
        printf("ID=N/A ");
    }
    printf(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ",robot.height(),robot.x(),robot.y());
    if (robot.has_orientation()) {
        printf("ANGLE=%6.3f ",robot.orientation());
    } else {
        printf("ANGLE=N/A    ");
    }
    printf("RAW=<%8.2f,%8.2f>\n",robot.pixel_x(),robot.pixel_y());
}

int main(int argc, char *argv[]){
    (void)argc;
    (void)argv;
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;

    GrSim_Client grSim_client;

    vector<myball> ballsvect; // Vetor de possíveis bolas usado no estado de decisão de qual bola é a real
    myball chosen_ball(0,0); // Bola escolhida como real
    myball tempball(0,0); // Bola temporária para push no vetor de bolas possíveis
    bool locked = 0; // Variável que sinaliza se a bola já foi detectada e é corretamente analizada
    bool firstunlocked = 1; // Variável que sinaliza se o sistema está no primeiro frame do estado de consolidação

    myrobot b_robotsvect[11]; // Array de robôs no time azul
    myrobot y_robotsvect[11]; // Array de robôs no time amarelo

    while(true) {
        if (client.receive(packet)) {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_detection()) {
                SSL_DetectionFrame detection = packet.detection();
                //Display the contents of the robot detection results:
                double t_now = GetTimeSec();

                printf("-[Detection Data]-------\n");
                //Frame info:
                printf("Camera ID=%d FRAME=%d T_CAPTURE=%.4f\n",detection.camera_id(),detection.frame_number(),detection.t_capture());

                printf("SSL-Vision Processing Latency                   %7.3fms\n",(detection.t_sent()-detection.t_capture())*1000.0);
                printf("Network Latency (assuming synched system clock) %7.3fms\n",(t_now-detection.t_sent())*1000.0);
                printf("Total Latency   (assuming synched system clock) %7.3fms\n",(t_now-detection.t_capture())*1000.0);
                int balls_n = detection.balls_size();
                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();

                //Variáveis temporárias
                int vectb_n = ballsvect.size(); // Tamanho do vetor de bolas "válidas"


                //Ball info:
                for (int i = 0; i < balls_n; i++) {

                    SSL_DetectionBall ball = detection.balls(i);
                    printf("-Ball (%2d/%2d): CONF=%4.2f POS=<%9.2f,%9.2f> ", i+1, balls_n, ball.confidence(),ball.x(),ball.y());
                    if (ball.has_z()) {
                        printf("Z=%7.2f ",ball.z());
                    } else {
                        printf("Z=N/A   ");
                    }
                    printf("RAW=<%8.2f,%8.2f>\n",ball.pixel_x(),ball.pixel_y());


                    //P�s-processamento do vetor de bolas

                    
                    if(ball.confidence() >= CONFLIM){ //Bolas com "confiança" menor que determinado limite são ignoradas
                        
                        if(ball.area() >= SUP_AREALIM && ball.area() <= INF_AREALIM){ // Bolas com área menores, ou muito maiores, do que o da bola real são ignoradas

                            if(locked){ //Estado consolidado
                                        //Aqui checkamos se a bola atual está na área de previsão da bola escolhida como real
                                        

                                float dx = ball.x()-chosenball.x();
                                float dy = ball.y()-chosenball.y();

                                dx*=dx;
                                dy*=dy;

                                if(dx+dy <= chosen_ball.predict_radius()){ //Caso haja mais de uma bola nessa área, a mais próxima do centro será escolhida como a bola real

                                    chosen_ball.detection(ball.x(), ball.y(), dx+dy);
                                }

                            }

                            else{ //Estado de consolidação 

                                if(firstunlocked){ //Primeiro frame no estado de consolidação
                                    //Basicamente pushamos todas as bolas válidas que existem no frame atual no vetor de bolas válidas
                                    
                                    tempball.reset(ball.x(), ball.y());

                                    ballsvect.push_back(tempball);
                                }
                                else{ // Outros frames no estado de consolidação

                                    for(int j = 0; j<vectb_n; j++){ // Checkamos se a bola atual está na área de previsão de alguma das bolas em ballsvect

                                        float dx = ball.x()-ballsvect[j].x();
                                        float dy = ball.y()-ballsvect[j].y();

                                        dx*=dx;
                                        dy*=dy;

                                        if(dx + dy <= ballsvect[j].predict_radius()){ //Como só podemos escolher uma delas, a mais próxima do centro dessa área será escolhida como a bola de índice j

                                            ballsvect[j].detection(ball.x(), ball.y(), dx+dy);
                                        }

                                    }
                                }    
                            }    
                        }
                    }
                }

                if(locked){

                    if(chosen_ball.detected()) // Se houve a ocorrência de alguma bola na área de previsão da bola escolhida no frame atual
                                              // atualizamos a posição e a velocidade de tal entidade 
                        chosen_ball.refresh_v();
                    
                    else{ // Se não...

                        if(chosen_ball.vanishcntr() < VNSH_LIM) // Se a bola ainda não estrapolou o limite de frames sem detecção, continuamos as previsões
                            chosen_ball.predict();
                            
                        else{ // Se o limite foi estrapolado, voltamos à fase de consolidação
                            locked = 0;
                            firstunlocked = 1;
                        }
                    }
                }
                else{
                    if(firstunlocked){ // Tratamento para o primeiro frame de consolidação, atualizando a quantidade de bolas válidas que possuímos e finalizando tal fase inicial
                        vectb_n = ballsvect.size();
                        firstunlocked = 0;
                    }
                    else{ 

                        for(int j = 0; j<vectb_n; j++){ // Aqui checkamos cada bola no vetor de bolas válidas para analisar se dentro de cada uma de suas áreas de previsão
                                                        // Houveram detecções

                            if(ballsvect[j].detected()){

                                // Se sim, atualizamos a posição e velocidade de tal entidade

                                ballsvect[j].refresh_v();

                                if(ballsvect[j].framecntr() >= FRM_LIM){
                                    
                                    // Mas se ocorrer de alguma das bolas dentro do vetor de bolas válidas ter tido uma previsão satisfatória (ter tido várias previsões bem sucedidas)
                                    // A promovemos à bola real e finalizamos a fase de consolidação, dando início à fase consolidada

                                    chosen_ball = ballsvect[j];
                                    ballsvect.clear();
                                    locked = 1;

                                    break;
                                }
                            }
                            
                            else{

                                //Se não teve detecção...

                                if(ballsvect[j].vanishcntr() < VNSH_LIM) // Se ela não estrapolou o limite de detecções mal sucedidas, predizemos sua próxima posição
                                    ballsvect[j].predict()

                                
                                else{
                                    ballsvect.erase(ballsvect.begin() + j); // Se o limite já foi excedido, apagamos tal bola do vetor de bolas
                                    vectb_n = ballsvect.size(); // Atualizamos a quantidade de bolas
                                    j--;
                                }
                            }
                        }

                        if(ballsvect.empty()) firstunlocked = 1; // Se o vetor de bolas ficar vazio, reiniciamos a fase de consolidação

                    }
                }

                //Blue robot info:
                for (int i = 0; i < robots_blue_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_blue(i);
                    printf("-Robot(B) (%2d/%2d): ",i+1, robots_blue_n);
                    printRobotInfo(robot);
                    if(robot.x() <= 0){
                        grSim_client.sendCommand(1.0, i);
                    }else{
                        grSim_client.sendCommand(-1.0, i);
                    }

                    if(robot.has_robot_id()){
                        // Atualizamos as posição e velocidade do robô atual detectado
                        b_robotsvect[robot.robot_id()%11].detection(robot.x(), robot.y());
                    }

                }
                for(int i = 0; i < 11; i++) // Aqui buscamos no array de robôs os que não foram detectados no frame atual, e então predizemos a próxima posição de cada um desses
                    if(!b_robotsvect[i].detected()){
                        b_robotsvect[i].predict();
                    }

                    b_robotsvect[i].undetect(); // Desmarcamento de todos os robôs para a análise no próximo frame
                }

                //Yellow robot info:
                for (int i = 0; i < robots_yellow_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_yellow(i);
                    printf("-Robot(Y) (%2d/%2d): ",i+1, robots_yellow_n);
                    printRobotInfo(robot);

                    if(robot.has_robot_id()){
                        // Atualizamos as posição e velocidade do robô atual detectado
                        y_robotsvect[robot.robot_id()%11].detection(robot.x(), robot.y());
                    }

                }
                for(int i = 0; i < 11; i++) { // Aqui buscamos no array de robôs os que não foram detectados no frame atual, e então predizemos a próxima posição de cada um desses
                    if(!y_robotsvect[i].detected()){
                        y_robotsvect[i].predict();
                    }

                    y_robotsvect[i].undetect(); // Desmarcamento de todos os robôs para a análise no próximo frame
                }

            }

            //see if packet contains geometry data:
            if (packet.has_geometry()) {
                const SSL_GeometryData & geom = packet.geometry();
                printf("-[Geometry Data]-------\n");

                const SSL_GeometryFieldSize & field = geom.field();
                printf("Field Dimensions:\n");
                printf("  -field_length=%d (mm)\n",field.field_length());
                printf("  -field_width=%d (mm)\n",field.field_width());
                printf("  -boundary_width=%d (mm)\n",field.boundary_width());
                printf("  -goal_width=%d (mm)\n",field.goal_width());
                printf("  -goal_depth=%d (mm)\n",field.goal_depth());
                printf("  -field_lines_size=%d\n",field.field_lines_size());
                printf("  -field_arcs_size=%d\n",field.field_arcs_size());

                int calib_n = geom.calib_size();
                for (int i=0; i< calib_n; i++) {
                    const SSL_GeometryCameraCalibration & calib = geom.calib(i);
                    printf("Camera Geometry for Camera ID %d:\n", calib.camera_id());
                    printf("  -focal_length=%.2f\n",calib.focal_length());
                    printf("  -principal_point_x=%.2f\n",calib.principal_point_x());
                    printf("  -principal_point_y=%.2f\n",calib.principal_point_y());
                    printf("  -distortion=%.2f\n",calib.distortion());
                    printf("  -q0=%.2f\n",calib.q0());
                    printf("  -q1=%.2f\n",calib.q1());
                    printf("  -q2=%.2f\n",calib.q2());
                    printf("  -q3=%.2f\n",calib.q3());
                    printf("  -tx=%.2f\n",calib.tx());
                    printf("  -ty=%.2f\n",calib.ty());
                    printf("  -tz=%.2f\n",calib.tz());

                    if (calib.has_derived_camera_world_tx() && calib.has_derived_camera_world_ty() && calib.has_derived_camera_world_tz()) {
                      printf("  -derived_camera_world_tx=%.f\n",calib.derived_camera_world_tx());
                      printf("  -derived_camera_world_ty=%.f\n",calib.derived_camera_world_ty());
                      printf("  -derived_camera_world_tz=%.f\n",calib.derived_camera_world_tz());
                    }

                }
            }
        }
    }

    return 0;
}