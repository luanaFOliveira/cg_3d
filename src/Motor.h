#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "gl_canvas2d.h"
#include "Vector3.h"

// typeRotate == 1 ->rotate X
// typeRotate == 2 ->rotate Y
// typeRotate == 3 ->rotate Z


class Motor{

    float angle = 0.0;
    float d = 131;
    float translateX,translateY;
    int typeRotate;
    float profundidade = 60;

    Vector3 entradaManibela[8];
    float xcManibela,ycManibela,largMenor,largMaior,altManibela;

    Vector3 entradaCirculoMaiorBiela[32];
    float xcCircMaior,ycCircMaior,raioMaior;

    Vector3 entradaCirculoMenorBiela[32];
    float xcCircMenor,ycCircMenor,raioMenor;

    float tamBiela;
    int segments = 16;
    float alturaCilindros = 40;



public:

    void defineValores(){

        xcManibela = 300;
        ycManibela = 700;
        largMenor = 30;
        largMaior = 60;
        altManibela = 120;
        tamBiela = 200;
        xcCircMaior = xcManibela;
        ycCircMaior = ycManibela - altManibela/2;
        xcCircMenor = xcCircMaior;
        ycCircMenor = ycCircMaior-tamBiela;
        raioMenor = 20;
        raioMaior = 40;
    }

    void criaVetores(){
        for (int i = 0; i < 8; i++) {
            entradaManibela[i] = *new Vector3();
        }

        for (int i = 0; i < 32; i++) {
            entradaCirculoMaiorBiela[i] = *new Vector3();
        }

        for (int i = 0; i < 32; i++) {
            entradaCirculoMenorBiela[i] = *new Vector3();
        }

    }

    void criaManibela(){

        entradaManibela[0].x = -largMenor/2;
        entradaManibela[0].y = -altManibela/2;
        entradaManibela[0].z = -profundidade/2;

        entradaManibela[1].x = largMenor/2;
        entradaManibela[1].y = -altManibela/2;
        entradaManibela[1].z = -profundidade/2;

        entradaManibela[2].x = largMaior/2;
        entradaManibela[2].y = altManibela/2;
        entradaManibela[2].z = -profundidade/2;

        entradaManibela[3].x = -largMaior/2;
        entradaManibela[3].y = altManibela/2;
        entradaManibela[3].z = -profundidade/2;

        entradaManibela[4].x = -largMenor/2;
        entradaManibela[4].y = -altManibela/2;
        entradaManibela[4].z = profundidade/2;

        entradaManibela[5].x = largMenor/2;
        entradaManibela[5].y = -altManibela/2;
        entradaManibela[5].z = profundidade/2;

        entradaManibela[6].x = largMaior/2;
        entradaManibela[6].y = altManibela/2;
        entradaManibela[6].z = profundidade/2;

        entradaManibela[7].x = -largMaior/2;
        entradaManibela[7].y = altManibela/2;
        entradaManibela[7].z = profundidade/2;
        /*
        entradaManibela[0].x = xcManibela-largMenor/2;
        entradaManibela[0].y = ycManibela-altManibela/2;
        entradaManibela[0].z = -profundidade/2;

        entradaManibela[1].x = xcManibela+largMenor/2;
        entradaManibela[1].y = ycManibela-altManibela/2;
        entradaManibela[1].z = -profundidade/2;

        entradaManibela[2].x = xcManibela+largMaior/2;
        entradaManibela[2].y = ycManibela+altManibela/2;
        entradaManibela[2].z = -profundidade/2;

        entradaManibela[3].x = xcManibela-largMaior/2;
        entradaManibela[3].y = ycManibela+altManibela/2;
        entradaManibela[3].z = -profundidade/2;

        entradaManibela[4].x = xcManibela-largMenor/2;
        entradaManibela[4].y = ycManibela-altManibela/2;
        entradaManibela[4].z = profundidade/2;

        entradaManibela[5].x = xcManibela+largMenor/2;
        entradaManibela[5].y = ycManibela-altManibela/2;
        entradaManibela[5].z = profundidade/2;

        entradaManibela[6].x = xcManibela+largMaior/2;
        entradaManibela[6].y = ycManibela+altManibela/2;
        entradaManibela[6].z = profundidade/2;

        entradaManibela[7].x = xcManibela-largMaior/2;
        entradaManibela[7].y = ycManibela+altManibela/2;
        entradaManibela[7].z = profundidade/2;
*/
    }

    void criaCirculoMaior(){

        for (int i = 0; i < segments; i++) {
          float ang = (2 * PI * i) / segments;  // ângulo do ponto atual

          // ponto que conectam da base inferior
          entradaCirculoMaiorBiela[i].x = (cos(ang) * raioMaior);
          entradaCirculoMaiorBiela[i].y = (sin(ang) * raioMaior);
          entradaCirculoMaiorBiela[i].z = -alturaCilindros / 2;

          // ponto que conectam da base superior
          entradaCirculoMaiorBiela[i + segments].x = (cos(ang) * raioMaior);
          entradaCirculoMaiorBiela[i + segments].y = (sin(ang) * raioMaior);
          entradaCirculoMaiorBiela[i + segments].z = alturaCilindros / 2;
        }

        /*
        for (int i = 0; i < segments; i++) {
          float ang = (2 * PI * i) / segments;  // ângulo do ponto atual

          // ponto que conectam da base inferior
          entradaCirculoMaiorBiela[i].x = xcCircMaior+cos(ang) * raioMaior;
          entradaCirculoMaiorBiela[i].y = ycCircMaior+sin(ang) * raioMaior;
          entradaCirculoMaiorBiela[i].z = -alturaCilindros / 2;

          // ponto que conectam da base superior
          entradaCirculoMaiorBiela[i + segments].x = xcCircMaior+cos(ang) * raioMaior;
          entradaCirculoMaiorBiela[i + segments].y = ycCircMaior+sin(ang) * raioMaior;
          entradaCirculoMaiorBiela[i + segments].z = alturaCilindros / 2;
        }

        */
    }

    void criaCirculoMenor(){
        for (int i = 0; i < segments; i++) {
          float ang = (2 * PI * i) / segments;  // ângulo do ponto atual

          // ponto que conectam da base inferior
          entradaCirculoMenorBiela[i].x = (cos(ang) * raioMenor);
          entradaCirculoMenorBiela[i].y = (sin(ang) * raioMenor);
          entradaCirculoMenorBiela[i].z = -alturaCilindros / 2;

          // ponto que conectam da base superior
          entradaCirculoMenorBiela[i + segments].x = (cos(ang) * raioMenor);
          entradaCirculoMenorBiela[i + segments].y = (sin(ang) * raioMenor);
          entradaCirculoMenorBiela[i + segments].z = alturaCilindros / 2;
        }
        /*
        for (int i = 0; i < segments; i++) {
          float ang = (2 * PI * i) / segments;  // ângulo do ponto atual

          // ponto que conectam da base inferior
          entradaCirculoMaiorBiela[i].x = xcCircMenor+cos(ang) * raioMenor;
          entradaCirculoMaiorBiela[i].y = ycCircMenor+sin(ang) * raioMenor;
          entradaCirculoMaiorBiela[i].z = -alturaCilindros / 2;

          // ponto que conectam da base superior
          entradaCirculoMaiorBiela[i + segments].x = xcCircMenor+cos(ang) * raioMenor;
          entradaCirculoMaiorBiela[i + segments].y = ycCircMenor+sin(ang) * raioMenor;
          entradaCirculoMaiorBiela[i + segments].z = alturaCilindros / 2;
        }
*/
    }

    Motor(float transX,float transY){
        xcManibela = transX;
        ycManibela = transY;
        criaVetores();
        defineValores();
        criaManibela();
        criaCirculoMaior();
        criaCirculoMenor();
    }

    float pontoCentralLadoMenorX(){
        return (entradaManibela[0].x+entradaManibela[1].x)/2.0;
    }

    float pontoCentralLadoMenorY(){
        return (entradaManibela[0].y+entradaManibela[1].y)/2.0;
    }


    void mudaPontos(){
        float pcX = pontoCentralLadoMenorX();
        float pcY = pontoCentralLadoMenorY();
        xcCircMaior = pcX;
        ycCircMaior = pcY;
        ycCircMenor = ycCircMaior-tamBiela;
        //criaCirculoMaior();
        //criaCirculoMenor();
    }


    void renderProject(Vector3 entrada[],int tam,Vector2 saida[]){
        Vector3 p;

        for (int i = 0; i < tam; i++) {
            p = entrada[i];
            if(typeRotate == 1){
                //p = rotatePointAroundXAxis(p, angle);
            }else if(typeRotate == 2){
                //p = rotatePointAroundYAxis(p, angle);
            }else if(typeRotate == 3){
                //p = rotatePointAroundZAxis(p, angle);
            }
            saida[i] = project(p, d);
        }

    }


    void Render(){

        Vector2 saidaMani[8];
        renderProject(entradaManibela,8,saidaMani);
        Vector2 saidaCircMaior[32];
        renderProject(entradaCirculoMaiorBiela,32,saidaCircMaior);
        Vector2 saidaCircMenor[32];
        renderProject(entradaCirculoMenorBiela,32,saidaCircMenor);

        /*
        Vector3 p;
        Vector2 saida[8];

        for (int i = 0; i < 8; i++) {
            p = entradaManibela[i];
            if(typeRotate == 1){
                //p = rotatePointAroundXAxis(p, angle);
            }else if(typeRotate == 2){
                //p = rotatePointAroundYAxis(p, angle);
            }else if(typeRotate == 3){
                //p = rotatePointAroundZAxis(p, angle);
            }
            saida[i] = project(p, d);
        }
*/
        //Vector2 *saidaCircMaior = renderProject(entradaCirculoMaiorBiela,32);
        //Vector2 *saidaCircMenor = renderProject(entradaCirculoMenorBiela,32);



        CV::translate(xcManibela, ycManibela);
        CV::color(0);
        desenhaParalelepipedo(saidaMani);
        //xcCircMaior = xcManibela;
        //ycCircMaior = ycManibela - altManibela/2;
        CV::translate(xcCircMaior, ycCircMaior);
        CV::color(0);
        desenhaCilindro(saidaCircMaior);
        //ycCircMenor = ycCircMenor - tamBiela;
        //CV::translate(xcCircMaior, ycCircMenor);
        //CV::color(0);
        //desenhaCilindro(saidaCircMenor);

        rotateVec3d(0.5,8,entradaManibela);
        CV::circleFill(pontoCentralLadoMenorX(),pontoCentralLadoMenorY(),10,30);
        //xcCircMaior = xcManibela;
        //ycCircMaior = ycManibela-altManibela/2;
        //ycCircMenor = ycCircMaior-tamBiela;
        //mudaPontos();

    }



private:

    void desenhaCilindro(Vector2 cylinder[]) {
        for (int i = 0; i < segments; i++) {
            int nextIndex = (i + 1) % segments;

            CV::line(cylinder[i].x, cylinder[i].y, cylinder[nextIndex].x, cylinder[nextIndex].y);

            CV::line(cylinder[i + segments].x, cylinder[i + segments].y, cylinder[nextIndex + segments].x, cylinder[nextIndex + segments].y);

            CV::line(cylinder[i].x, cylinder[i].y, cylinder[i + segments].x, cylinder[i + segments].y);
        }
        for (int i = 0; i < segments; i++) {
            CV::point(cylinder[i].x, cylinder[i].y);

            CV::point(cylinder[i + segments].x, cylinder[i + segments].y);
        }

        CV::line(cylinder[segments - 1].x, cylinder[segments - 1].y, cylinder[0].x, cylinder[0].y);
        CV::line(cylinder[segments - 1 + segments].x, cylinder[segments - 1 + segments].y, cylinder[segments].x, cylinder[segments].y);

    }

    void desenhaParalelepipedo(Vector2 para[8]) {
       int secCount = 5;
       for (int i = 1; i < 4; i++) {
          //quadrado da frente
          CV::line(para[i-1].x, para[i-1].y, para[i].x, para[i].y);
          //quadrado do fundo
          CV::line(para[secCount - 1].x, para[secCount - 1].y, para[secCount].x, para[secCount].y);
          secCount++;
       }
       CV::line(para[3].x, para[3].y, para[0].x, para[0].y);
       CV::line(para[7].x, para[7].y, para[4].x, para[4].y);

       // arestas faltantes
       for (int i = 0; i < 4; i++) {
          CV::line(para[i].x, para[i].y, para[i+4].x, para[i+4].y);
       }
    }

    void rotateVec3d(float angulo, int n,Vector3 entrada[]) {

        double theta = angulo * PI / 180.0;
        float cx = calculaCentroX(entrada,n);
        float cy = calculaCentroY(entrada,n);
        float cz = calculaCentroZ(entrada,n);

        float c = cos(theta);
        float s = sin(theta);
        float rotationMatrix[3][3] = {
            {c, -s, 0},
            {s, c, 0},
            {0, 0, 1}
        };

        for (int i = 0; i < n; i++) {
            float x = entrada[i].x - cx;
            float y = entrada[i].y - cy;
            float z = entrada[i].z - cz;

            float new_x = rotationMatrix[0][0] * x + rotationMatrix[0][1] * y + rotationMatrix[0][2] * z;
            float new_y = rotationMatrix[1][0] * x + rotationMatrix[1][1] * y + rotationMatrix[1][2] * z;
            float new_z = rotationMatrix[2][0] * x + rotationMatrix[2][1] * y + rotationMatrix[2][2] * z;

            //vetor[i].set(new_x + cx,new_y + cy,new_z + cz);
            entrada[i].x = new_x + cx;
            entrada[i].y = new_y + cy;
            entrada[i].z = new_z + cz;



        }
    }

    float calculaCentroX(Vector3 *vetor,int n){
        float sumx=0;
        for(int i =0;i<n;i++){
            sumx += vetor[i].x;

        }

        return sumx/n;
    }


    float calculaCentroY(Vector3 *vetor,int n){
        float sumx=0;
        for(int i =0;i<n;i++){
            sumx += vetor[i].y;

        }

        return sumx/n;
    }


    float calculaCentroZ(Vector3 *vetor,int n){
        float sumx=0;
        for(int i =0;i<n;i++){
            sumx += vetor[i].z;

        }

        return sumx/n;
    }



    Vector2 project(Vector3 v, float d) {
       Vector2 v2;
       v2.x = v.x * d / (v.z + d);
       v2.y = v.y * d / (v.z + d);

       return v2;
    }


    Vector3 rotatePointAroundYAxis(Vector3 p, double angle) {
        Vector3 rotatedPoint;

        // Conversão do ângulo de graus para radianos
        double theta = angle * PI / 180.0;

        // Matriz de rotação em torno do eixo y
        double rotationMatrix[3][3] = {
            {cos(theta), 0, sin(theta)},
            {0, 1, 0},
            {-sin(theta), 0, cos(theta)}
        };

        // Realiza a multiplicação da matriz de rotação pelo ponto
        rotatedPoint.x = rotationMatrix[0][0] * p.x + rotationMatrix[0][1] * p.y + rotationMatrix[0][2] * p.z;
        rotatedPoint.y = rotationMatrix[1][0] * p.x + rotationMatrix[1][1] * p.y + rotationMatrix[1][2] * p.z;
        rotatedPoint.z = rotationMatrix[2][0] * p.x + rotationMatrix[2][1] * p.y + rotationMatrix[2][2] * p.z;

        return rotatedPoint;
    }



    Vector3 rotatePointAroundXAxis(Vector3 p, double angle) {
        Vector3 rotatedPoint;

        // Conversão do ângulo de graus para radianos
        double theta = angle * PI / 180.0;

        // Matriz de rotação em torno do eixo y
        double rotationMatrix[3][3] = {
            {1, 0, 0},
            {0,cos(theta),sin(theta)},
            {0,-sin(theta),cos(theta)}
        };

        // Realiza a multiplicação da matriz de rotação pelo ponto
        rotatedPoint.x = rotationMatrix[0][0] * p.x + rotationMatrix[0][1] * p.y + rotationMatrix[0][2] * p.z;
        rotatedPoint.y = rotationMatrix[1][0] * p.x + rotationMatrix[1][1] * p.y + rotationMatrix[1][2] * p.z;
        rotatedPoint.z = rotationMatrix[2][0] * p.x + rotationMatrix[2][1] * p.y + rotationMatrix[2][2] * p.z;

        return rotatedPoint;
    }


    Vector3 rotatePointAroundZAxis(Vector3 p, double angle) {
        Vector3 rotatedPoint;

        // Conversão do ângulo de graus para radianos
        double theta = angle * PI / 180.0;

        // Matriz de rotação em torno do eixo y
        double rotationMatrix[3][3] = {
            {cos(theta),sin(theta),0},
            {-sin(theta),cos(theta),0},
            {0, 0, 1},
        };

        // Realiza a multiplicação da matriz de rotação pelo ponto
        rotatedPoint.x = rotationMatrix[0][0] * p.x + rotationMatrix[0][1] * p.y + rotationMatrix[0][2] * p.z;
        rotatedPoint.y = rotationMatrix[1][0] * p.x + rotationMatrix[1][1] * p.y + rotationMatrix[1][2] * p.z;
        rotatedPoint.z = rotationMatrix[2][0] * p.x + rotationMatrix[2][1] * p.y + rotationMatrix[2][2] * p.z;

        return rotatedPoint;
    }




};

#endif




