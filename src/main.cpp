
#include <GL/glut.h>
#include <GL/freeglut_ext.h> //callback da wheel do mouse.

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include "gl_canvas2d.h"
#include "Cilindro.h"
#include "Pistao.h"
#include "Biela.h"
#include "Manibela.h"
#include "Motor.h"
#include "Trapezio.h"
using namespace std;

#define X_AXIS_LEN 400
#define Y_AXIS_LEN 400

//Cilindro* cilindro;
Pistao* pistao;
Biela* biela;
Manibela* manibela;
Motor* motor;
Trapezio* trap;

Vector3 entradaCubo[8];
Vector3 entradaTriangulo[4];
Vector3 entradaCilindro[32];  // matriz para armazenar os pontos
Vector3 entradaCilindro2[36000];
Vector3 entradaPoligono[64]; // matriz para armazenar os pontos

float radiusTop = 20.0;    // raio do círculo de cima
float radiusBottom = 30.0; // raio do círculo de baixo
float height = 50.0;       // altura do polígono
int segments = 16;

float x = 100;
float y = 100;
float z = 50;
float d = 131;

float width = 100;
//float height = 100;
float deth = 100;

float angPonteiro = 0.0;

float manix[4],maniy[4];
float largMenor = 60,largMaior = 140,altMani =140;
float xcmani = 300,ycmani = 500;
float cCircGraX = 300,cCircGraY = ycmani-altMani/2;
float cCircPeqY =cCircGraY-200;
float cCircPeqX = cCircGraX-(cCircPeqY/tan(PI/4));
float bx[4],by[4];
float raio1 = 20,raio2 = 40;
float px[4],py[4];
float xcpistao = cCircPeqX,ycpistao = cCircPeqY-raio1;
float alt = 80,larg = 80;
float qx[4],qy[4];
float tamQ = raio2+10;


//variavel global para selecao do que sera exibido na canvas.
int opcao  = 50;
int screenWidth = 1400, screenHeight = 900; //largura e altura inicial da tela . Alteram com o redimensionamento de tela.
int mouseX, mouseY;           //variaveis globais do mouse para poder exibir dentro da render().


// copia uma array para o outro
void arrcopy(float *origin, float* dest, int tam){
   for (int i = 0; i < tam; i++) {
      dest[i] = origin[i];
   }
}


// translada um quadrado para algum ponto. offset define o pivot
void translateSquare(float *qx, float *qy, float pivotX, float pivotY) {
   for (int i = 0; i < 4; i++) {
      qx[i] += pivotX;
      qy[i] += pivotY;
   }
}

float getCenterX(float * vx, int nPoints) {
   float sum = 0;
   for (int i = 0; i < nPoints; i++) {
      sum += vx[i];
   }

   return sum / nPoints;
}

float getCenterY(float * vy, int nPoints) {
   float sum = 0;
   for (int i = 0; i < nPoints; i++) {
      sum += vy[i];
   }

   return sum / nPoints;
}

void rotatePoint(float& x1, float& y1, float angle) {
   // Converte o ângulo para radianos
   float rad = (angle) * PI / 180.0;

   // Rotaciona o ponto utilizando a matriz de rotação
   float newX = x1 * cos(rad) + y1 * (-sin(rad));
   float newY = x1 * sin(rad) + y1 * cos(rad);

   x1 = newX;
   y1 = newY;
}


// rotaciona todos os pontos de um rect
void rotateRect(float* qx, float* qy, float angle) {
   for (int i = 0; i < 4; i++) {
      rotatePoint(qx[i], qy[i], angle);
   }
}

Vector2 project(Vector3 v, float d) {
   Vector2 v2;
   v2.x = v.x * d / (v.z + d);
   v2.y = v.y * d / (v.z + d);

   return v2;
}



void drawCube(Vector2 cube[8]) {
   int secCount = 5;
   for (int i = 1; i < 4; i++) {
      //quadrado da frente
      CV::line(cube[i-1].x, cube[i-1].y, cube[i].x, cube[i].y);
      //quadrado do fundo
      CV::line(cube[secCount - 1].x, cube[secCount - 1].y, cube[secCount].x, cube[secCount].y);
      secCount++;
   }
   CV::line(cube[3].x, cube[3].y, cube[0].x, cube[0].y);
   CV::line(cube[7].x, cube[7].y, cube[4].x, cube[4].y);

   // arestas faltantes
   for (int i = 0; i < 4; i++) {
      CV::line(cube[i].x, cube[i].y, cube[i+4].x, cube[i+4].y);
   }
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
        {0, 1, 0},
        {cos(theta), 0, sin(theta)},
        {-sin(theta), 0, cos(theta)}
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
        {cos(theta), 0, sin(theta)},
        {-sin(theta), 0, cos(theta)},
        {0, 1, 0},
    };

    // Realiza a multiplicação da matriz de rotação pelo ponto
    rotatedPoint.x = rotationMatrix[0][0] * p.x + rotationMatrix[0][1] * p.y + rotationMatrix[0][2] * p.z;
    rotatedPoint.y = rotationMatrix[1][0] * p.x + rotationMatrix[1][1] * p.y + rotationMatrix[1][2] * p.z;
    rotatedPoint.z = rotationMatrix[2][0] * p.x + rotationMatrix[2][1] * p.y + rotationMatrix[2][2] * p.z;

    return rotatedPoint;
}




void desenhaTriangulo3d(Vector2 tri[4]) {

   for (int i = 1; i < 3; i++) {
      //triangulo da frente
      CV::line(tri[i-1].x, tri[i-1].y, tri[i].x, tri[i].y);

   }

   CV::line(tri[2].x, tri[2].y, tri[0].x, tri[0].y);
   CV::line(tri[0].x, tri[0].y, tri[3].x, tri[3].y);
   CV::line(tri[1].x, tri[1].y, tri[3].x, tri[3].y);
   CV::line(tri[2].x, tri[2].y, tri[3].x, tri[3].y);




}
/*
void desenhaCilindro3d(Vector2 cili1[],Vector2 cili2[]) {
    int passo = 1000;
   for (int i = 0; i < n; i++) {
      //triangulo da frente
        CV::color(0,0,0);
        CV::point(cili1[i].x,cili1[i].y);
        CV::point(cili2[i].x,cili2[i].y);
        //CV::line(cili1[i+passo].x,cili1[i+passo].y,cili2[i+passo].x,cili2[i+passo].y);

   }
   for (int i = 0; i < n; i+=passo) {
      //triangulo da frente

        CV::line(cili1[i].x,cili1[i].y,cili2[i].x,cili2[i].y);

   }


}
*/
void drawCylinder(Vector2 cylinder[]) {
  // desenhar as linhas laterais do cilindro
  for (int i = 0; i < segments; i++) {
    int nextIndex = (i + 1) % segments;

    // linha da base inferior
    CV::line(cylinder[i].x, cylinder[i].y, cylinder[nextIndex].x, cylinder[nextIndex].y);

    // linha da base superior
    CV::line(cylinder[i + segments].x, cylinder[i + segments].y, cylinder[nextIndex + segments].x, cylinder[nextIndex + segments].y);

    // linha vertical que conecta as bases
    CV::line(cylinder[i].x, cylinder[i].y, cylinder[i + segments].x, cylinder[i + segments].y);
  }
  for (int i = 0; i < segments; i++) {
    // Base inferior
    CV::point(cylinder[i].x, cylinder[i].y);

    // Base superior
    CV::point(cylinder[i + segments].x, cylinder[i + segments].y);
  }
  /*

  // desenhar as linhas das bases do cilindro
  for (int i = 1; i < segments; i++) {
    // base inferior
    CV::line(cylinder[i - 1].x, cylinder[i - 1].y, cylinder[i].x, cylinder[i].y);
    // base superior
    CV::line(cylinder[i - 1 + segments].x, cylinder[i - 1 + segments].y, cylinder[i + segments].x, cylinder[i + segments].y);
  }
*/
  // conectar o último ponto com o primeiro ponto para fechar as bases
  CV::line(cylinder[segments - 1].x, cylinder[segments - 1].y, cylinder[0].x, cylinder[0].y);
  CV::line(cylinder[segments - 1 + segments].x, cylinder[segments - 1 + segments].y, cylinder[segments].x, cylinder[segments].y);

}
void drawPolygon(Vector2 cylinder[]) {
  // Desenhar as linhas laterais do cilindro
  for (int i = 0; i < segments; i++) {
    int nextIndex = (i + 1) % segments;

    // Linhas das bases
    CV::line(cylinder[i].x, cylinder[i].y, cylinder[nextIndex].x, cylinder[nextIndex].y);
    CV::line(cylinder[i + segments].x, cylinder[i + segments].y, cylinder[nextIndex + segments].x, cylinder[nextIndex + segments].y);

    // Linhas verticais que conectam as bases
    CV::line(cylinder[i].x, cylinder[i].y, cylinder[i + segments].x, cylinder[i + segments].y);
  }

  // Desenhar as linhas das bases do cilindro
  for (int i = 1; i < segments; i++) {
    // Base superior
    CV::line(cylinder[i - 1].x, cylinder[i - 1].y, cylinder[i].x, cylinder[i].y);

    // Base inferior
    CV::line(cylinder[i - 1 + segments].x, cylinder[i - 1 + segments].y, cylinder[i + segments].x, cylinder[i + segments].y);
  }

  // Conectar o último ponto com o primeiro ponto para fechar as bases
  CV::line(cylinder[segments - 1].x, cylinder[segments - 1].y, cylinder[0].x, cylinder[0].y);
  CV::line(cylinder[3 * segments - 1].x, cylinder[3 * segments - 1].y, cylinder[2 * segments].x, cylinder[2 * segments].y);
}

void desenhaModeloBasicoPistao(){
    //circulo
    float raio = 50;
    float ang = 0;
    for( ang = 0; ang <= 2 * PI ; ang +=0.01)
   {
        float x1 = raio * cos(ang);
        float y1 = raio * sin (ang);
        CV::color(1,0,0);
        CV::point(x1,y1);

   }
    //ponteiro
    float xc = 0;
    float yc = 0;
    float xp = 50 * cos(angPonteiro);
    float yp = 50 * sin (angPonteiro);
    CV::color(0,0,0);
    CV::line(xc, yc, xp, yp);
    angPonteiro+=0.05;

    //linhacima
    //biela
    float xl = 0;
    float yl = yp-200;
    CV::color(0,0,0);
    CV::line(xp, yp, xl, yl);
    //quadrado na ponta
    //pistao
    float altq = 30;
    float ypm = yl;
    float xpm = 0;
    float xq[4],yq[4];
    xq[0] = xpm-altq/2;
    xq[1] = xpm+altq/2;
    xq[2] = xpm+altq/2;
    xq[3] = xpm-altq/2;

    yq[0] = ypm-altq;
    yq[1] = ypm-altq;
    yq[2] = ypm;
    yq[3] = ypm;
    CV::color(0,0,1);
    CV::polygon(xq,yq,4);

}

float calculaCentroX2d(int n,float *vx){
    float sumx=0;
    for(int i =0;i<n;i++){
        sumx += vx[i];

    }

    return sumx/n;
}


float calculaCentroY2d(int n,float *vy){
    float sumy=0;
    for(int i =0;i<n;i++){
        sumy += vy[i];

    }

    return sumy/n;
}


void rotateVec2d(float *vx,float *vy,float angulo,int n){

    double theta = angulo * PI / 180.0;
    float cx = calculaCentroX2d(n,vx);
    float cy = calculaCentroY2d(n,vy);

    float c = cos(theta);
    float s = sin(theta);
    float matris[2][2] = {{c, -s}, {s, c}};

    for (int i = 0; i < n; i++) {
        float x = vx[i] - cx;
        float y = vy[i] - cy;

        float new_x = matris[0][0] * x + matris[0][1] * y;
        float new_y = matris[1][0] * x + matris[1][1] * y;

        vx[i] = new_x + cx;
        vy[i] = new_y + cy;

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

void rotateVec3d(Vector3 vetor[], float angulo, int n) {

    double theta = angulo * PI / 180.0;
    float cx = calculaCentroX(vetor,n);
    float cy = calculaCentroY(vetor,n);
    float cz = calculaCentroZ(vetor,n);

    float c = cos(theta);
    float s = sin(theta);
    float rotationMatrix[3][3] = {
        {c, -s, 0},
        {s, c, 0},
        {0, 0, 1}
    };

    for (int i = 0; i < n; i++) {
        float x = vetor[i].x - cx;
        float y = vetor[i].y - cy;
        float z = vetor[i].z - cz;

        float new_x = rotationMatrix[0][0] * x + rotationMatrix[0][1] * y + rotationMatrix[0][2] * z;
        float new_y = rotationMatrix[1][0] * x + rotationMatrix[1][1] * y + rotationMatrix[1][2] * z;
        float new_z = rotationMatrix[2][0] * x + rotationMatrix[2][1] * y + rotationMatrix[2][2] * z;

        vetor[i].set(new_x + cx,new_y + cy,new_z + cz);
        //vetor[i].x = new_x + cx;
        //vetor[i].y = new_y + cy;
        //vetor[i].z = new_z + cz;
    }
}


void rotateBiela2d(float angulo,float *vx,float *vy,float n){

    //rotateVec2d(bx,by,angulo,4);
    //double theta = angulo * PI / 180.0;
    //cCircGraX = raio2 * cos(theta);
    //cCircGraY = raio2 * sin(theta);

    double theta = angulo * PI / 180.0;
    float cx = xcmani;
    float cy = ycmani;

    float c = cos(theta);
    float s = sin(theta);
    float matris[2][2] = {{c, -s}, {s, c}};

    for (int i = 0; i < n; i++) {

        float x = vx[i] - cx;
        float y = vy[i] - cy;

        float new_x = matris[0][0] * x + matris[0][1] * y;
        float new_y = matris[1][0] * x + matris[1][1] * y;

        vx[i] = new_x + cx;
        vy[i] = new_y + cy;

    }


}

void modelagem2dRender(){
    //biela em 2d
    bx[0] = cCircPeqX-raio1;
    bx[1] = cCircPeqX+raio1;
    bx[2] = cCircGraX+raio2;
    bx[3] = cCircGraX-raio2;
    by[0] = cCircPeqY;
    by[1] = cCircPeqY;
    by[2] = cCircGraY;
    by[3] = cCircGraY;
    CV::color(0,0,0);
    CV::polygonFill(bx,by,4);
/*
    qx[0] = cCircGraX-tamQ;
    qx[1] = cCircGraX+tamQ;
    qx[2] = cCircGraX+tamQ;
    qx[3] = cCircGraX-tamQ;
    qy[0] = cCircGraY-tamQ;
    qy[1] = cCircGraY-tamQ;
    qy[2] = cCircGraY+tamQ;
    qy[3] = cCircGraY+tamQ;
    CV::color(0,0,0);
    CV::polygonFill(qx,qy,4);
*/

    //circulo menor
    CV::color(0,0,0);
    CV::circleFill(cCircPeqX,cCircPeqY,raio1,30);
    CV::color(1,1,1);
    CV::circleFill(cCircPeqX,cCircPeqY,raio1-10,30);
    //CV::color(1,0,0);
    //CV::circleFill(cCircPeqX,cCircPeqY,10,30);

    //circulo maior
    CV::color(0,0,0);
    CV::circleFill(cCircGraX,cCircGraY,raio2,30);
    CV::color(1,1,1);
    CV::circleFill(cCircGraX,cCircGraY,raio2-10,30);

    //pistao em 2d
    px[0] = xcpistao-larg/2;
    px[1] = xcpistao+larg/2;
    px[2] = xcpistao+larg/2;
    px[3] = xcpistao-larg/2;
    py[0] = ycpistao-alt;
    py[1] = ycpistao-alt;
    py[2] = ycpistao;
    py[3] = ycpistao;
    rotateVec2d(px,py,45,4);
    CV::color(0,0,1);
    CV::polygonFill(px,py,4);

    CV::color(1,1,1);
    CV::circleFill(xcpistao,ycpistao-alt/2,10,30);
    //manibela

    CV::color(1,0,0);
    CV::polygonFill(manix,maniy,4);
    CV::color(1,1,1);
    CV::circleFill(xcmani,ycmani,30,30);
    rotateVec2d(manix,maniy,0.8,4);

    cCircGraX = (manix[0]+manix[1])/2.0;
    cCircGraY = (maniy[0]+maniy[1])/2.0;
    cCircPeqY = cCircGraY-(200/sin(PI/4));
    //cCircPeqX = (cCircPeqY/tan(PI/4));
    cCircPeqX = (cCircPeqY*cos(PI/4));
    xcpistao = cCircPeqX;
    ycpistao = cCircPeqY;


}

void modelagem2dCriacao(){
    manix[0] = xcmani-largMenor/2;
    manix[1] = xcmani+largMenor/2;
    manix[2] = xcmani+largMaior/2;
    manix[3] = xcmani-largMaior/2;
    maniy[0] = ycmani-altMani/2;
    maniy[1] = ycmani-altMani/2;
    maniy[2] = ycmani+altMani/2;
    maniy[3] = ycmani+altMani/2;


}

void modelagem3dRender(){
    biela->Render();
    manibela->Render();
    //pistao->Render();
    manibela->rotacionaManibela();
    float tempcCircGraX = manibela->getPointXCenter();
    float tempcCircGraY = manibela->getPointYCenter();
    biela->moveBiela(tempcCircGraX,tempcCircGraY);
    //pistao->movePistao(biela->getYcentralCirculoMenor());

}

//funcao chamada continuamente. Deve-se controlar o que desenhar por meio de variaveis globais
//Todos os comandos para desenho na canvas devem ser chamados dentro da render().
//Deve-se manter essa fun  o com poucas linhas de codigo.
float angle = 0.0;
void render()
{

    //motor->Render();

    //modelagem basica funcionando
    //desenhaModeloBasicoPistao();

    //cilindro->Render();
    //pistao->Render();

   // CV::translate(300,100);
    //modelagem2dRender();
    modelagem3dRender();


       // número de segmentos para cada círculo
       /*
    Vector3 p;
   Vector2 saidaPoligono[64];

   for (int i = 0; i < 64; i++) {
      p = entradaPoligono[i];
      p = rotatePointAroundYAxis(p, angle);
      saidaPoligono[i] = project(p, d);
   }

   CV::translate(300, 200);
   CV::color(0);

   angle += 0.3;
   drawPolygon(saidaPoligono);
*/


    /*
    float raioMani = 100,xcCircMani = xcmani,ycCircMani=ycmani-altMani/2;
    for(int j=0;j<PI;j+=0.01){
        float tempx = raioMani * cos(j);
        float tempy = raioMani * sin (j);
        CV::color(0,0,0);
        CV::point(tempx,tempy);
    }
*/
    /*
    Vector3 p;
   Vector2 saidaCilindro[32];

   for (int i = 0; i < 32; i++) {
      p = entradaCilindro[i];
      p = rotatePointAroundYAxis(p, angle);
      saidaCilindro[i] = project(p, d);
   }

   CV::translate(300, 200);
   CV::color(0);

   angle += 0.3;
   drawCylinder(saidaCilindro);
*/
    /*
    //tentativa cilindro
    Vector3 p1;
    Vector3 p2;
    Vector2 saidaCilindro[n];
    Vector2 saidaCilindro2[n];

    for (int i = 0; i < n; i++) {
        p1 = entradaCilindro[i];
        p2 = entradaCilindro2[i];
        p1 = rotatePointAroundYAxis(p1, angle);
        p2 = rotatePointAroundYAxis(p2, angle);
        saidaCilindro[i] = project(p1, d);
        saidaCilindro2[i] = project(p2, d);
    }

    CV::translate(300, 300);
    CV::color(0);

    angle += 0.3;
    desenhaCilindro3d(saidaCilindro,saidaCilindro2);
*/
    /*
    //Vector3 p2;
    Vector2 saidaCilindro2[n];

    for (int i = 0; i < n; i++) {
      p2 = entradaCilindro2[i];
      p2 = rotatePointAroundYAxis(p2, angle);
      saidaCilindro2[i] = project(p2, d);
    }

    CV::translate(300, 500);
    CV::color(0);

    angle += 0.3;
    desenhaCilindro3d(saidaCilindro2);
*/
    /*
   Vector3 p;
   Vector2 saidaCubo[8];

   for (int i = 0; i < 8; i++) {
      p = entradaCubo[i];
      p = rotatePointAroundYAxis(p, angle);
      saidaCubo[i] = project(p, d);
   }

   CV::translate(300, 200);
   CV::color(0);

   angle += 0.3;
   drawCube(saidaCubo);

    /////////////////////////////
    Vector3 p2;
   Vector2 saidaTriangulo[4];

   for (int i = 0; i < 4; i++) {
      p2 = entradaTriangulo[i];
      p2 = rotatePointAroundYAxis(p2, angle);
      saidaTriangulo[i] = project(p2, d);
   }

   CV::translate(700, 200);
   CV::color(0);
   // eixos coordenados

   angle += 0.3;
   desenhaTriangulo3d(saidaTriangulo);
   */
}

//funcao chamada toda vez que uma tecla for pressionada.
void keyboard(int key)
{
   if( key < 200 )
   {
      opcao = key;
   }

   switch(key)
   {
   }
}

//funcao chamada toda vez que uma tecla for liberada
void keyboardUp(int key)
{
   switch(key)
   {

   }
}


//funcao para tratamento de mouse: cliques, movimentos e arrastos
void mouse(int button, int state, int wheel, int direction, int x, int y)
{
   mouseX = x; //guarda as coordenadas do mouse para exibir dentro da render()
   mouseY = y;

   if(state == 1) {

   };

   if( state == 0 ) //clicou
   {
   }
}

int main(void)
{
    motor = new Motor(300,500);
    modelagem2dCriacao();
    //cilindro = new Cilindro(100,50,200,200,3);
    biela = new Biela();
    pistao = new Pistao();
    manibela = new Manibela();
    trap = new Trapezio(200,50,100,20,500,500,2);

    for (int i = 0; i < 64; i++) {
        entradaPoligono[i] = *new Vector3();
    }

     for (int i = 0; i < segments; i++) {
        float angle = (2 * PI * i) / segments;  // ângulo do ponto atual

        // Pontos do cilindro superior
        entradaPoligono[i].x = cos(angle) * radiusTop;
        entradaPoligono[i].y = height / 2;
        entradaPoligono[i].z = sin(angle) * radiusTop;

        // Pontos do cilindro inferior
        entradaPoligono[i + segments].x = cos(angle) * radiusBottom;
        entradaPoligono[i + segments].y = -height / 2;
        entradaPoligono[i + segments].z = sin(angle) * radiusBottom;

        // Pontos que conectam os cilindros
        entradaPoligono[i + 2 * segments].x = entradaPoligono[i].x;
        entradaPoligono[i + 2 * segments].y = entradaPoligono[i].y;
        entradaPoligono[i + 2 * segments].z = entradaPoligono[i].z;

        entradaPoligono[i + 3 * segments].x = entradaPoligono[i + segments].x;
        entradaPoligono[i + 3 * segments].y = entradaPoligono[i + segments].y;
        entradaPoligono[i + 3 * segments].z = entradaPoligono[i + segments].z;
    }



    /*
    //tentativa gepeto cilindro
    float radius = 50.0;   // raio do cilindro
    float height = 40.0;  // altura do cilindro

    for (int i = 0; i < 32; i++) {
        entradaCilindro[i] = *new Vector3();
    }

    for (int i = 0; i < segments; i++) {
      float angle = (2 * PI * i) / segments;  // ângulo do ponto atual

      // ponto da base inferior
      entradaCilindro[i].x = cos(angle) * radius;
      entradaCilindro[i].y = sin(angle) * radius;
      entradaCilindro[i].z = -height / 2;

      // ponto da base superior
      entradaCilindro[i + segments].x = cos(angle) * radius;
      entradaCilindro[i + segments].y = sin(angle) * radius;
      entradaCilindro[i + segments].z = height / 2;
    }
    */
    /*
    //minha tentativa cilindro
    int raio = 100;
    for (int i = 0; i < n; i++) {
        entradaCilindro[i] = *new Vector3();
        entradaCilindro2[i] = *new Vector3();
    }
    float ang = 0.0;
    for (int i = 0; i < n; i++) {
        float x1 = raio * cos(ang);
        float y1 = raio * sin (ang);

        entradaCilindro[i].x = x1;
        entradaCilindro[i].y = y1;
        entradaCilindro[i].z = deth/2;
        entradaCilindro2[i].x = x1;
        entradaCilindro2[i].y = y1;
        entradaCilindro2[i].z = -deth/2;
        ang += 0.01;

    }
*/



    /*
   for (int i = 0; i < 8; i++) {
      entradaCubo[i] = *new Vector3();
   }

   entradaCubo[0].x = -width/2;
   entradaCubo[0].y = -height/2;
   entradaCubo[0].z = -deth/2;

   entradaCubo[1].x = width/2;
   entradaCubo[1].y = -height/2;
   entradaCubo[1].z = -deth/2;

   entradaCubo[2].x = width/2;
   entradaCubo[2].y = height/2;
   entradaCubo[2].z = -deth/2;

   entradaCubo[3].x = -width/2;
   entradaCubo[3].y = height/2;
   entradaCubo[3].z = -deth/2;

   entradaCubo[4].x = -width/2;
   entradaCubo[4].y = -height/2;
   entradaCubo[4].z = deth/2;

   entradaCubo[5].x = width/2;
   entradaCubo[5].y = -height/2;
   entradaCubo[5].z = deth/2;

   entradaCubo[6].x = width/2;
   entradaCubo[6].y = height/2;
   entradaCubo[6].z = deth/2;

   entradaCubo[7].x = -width/2;
   entradaCubo[7].y = height/2;
   entradaCubo[7].z = deth/2;

   for (int i = 0; i < 4; i++) {
      entradaTriangulo[i] = *new Vector3();
   }

   entradaTriangulo[0].x = width;
   entradaTriangulo[0].y = -height/2;
   entradaTriangulo[0].z = -deth/2;

   entradaTriangulo[1].x = width/2;
   entradaTriangulo[1].y = height/2;
   entradaTriangulo[1].z = deth/2;

   entradaTriangulo[2].x = -width/2;
   entradaTriangulo[2].y = height/2;
   entradaTriangulo[2].z = deth/2;

   entradaTriangulo[3].x = width;
   entradaTriangulo[3].y = height;
   entradaTriangulo[3].z = deth/2;

*/
   CV::init(&screenWidth, &screenHeight, "");
   CV::run();
}
