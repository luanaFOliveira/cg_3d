#ifndef __MANIBELA_H__
#define __MANIBELA_H__

#include "gl_canvas2d.h"
#include "Cilindro.h"
#include "Paralelepipedo.h"
#include "Trapezio.h"

class Manibela{

    Cilindro* circuloCentral;
    Trapezio* corpoTrapezio;

    float xcCirc,ycCirc,xcTrap,ycTrap,largMenor,largMaior,raioCirc;


public:

    Manibela(){

        xcCirc = 300;
        ycCirc = 700;
        xcTrap = xcCirc;
        ycTrap = ycCirc;
        raioCirc = 20;
        largMenor = 30;
        largMaior = 60;
        circuloCentral = new Cilindro(60,raioCirc,xcCirc,ycCirc,1);
        corpoTrapezio = new Trapezio(largMaior+60,largMenor,largMaior,60,xcTrap,ycTrap,3);
    }

    void rotacionaManibela(){
        corpoTrapezio->rotateVec3d(2.0,8);

    }

    Vector3 getPointTrapezio(int index){
        corpoTrapezio->getPoint(index);
    }

    float getPointXCenter(){
        return corpoTrapezio->getPointXCentroLadoMenor();
    }

    float getPointYCenter(){
        return corpoTrapezio->getPointYCentroLadoMenor();
    }


    void Render(){
        circuloCentral->Render();
        corpoTrapezio->Render();
        /*
        Vector3 p0 = corpoTrapezio->getPoint(0);
        Vector3 p1 = corpoTrapezio->getPoint(1);
        cCircGraX = (p0.x+p1.x)/2.0;
        cCircGraY = (p0.y+p1.y)/2.0;
        cCircPeqY = cCircGraY-200;
        */
    }

    //getters
    float getXcentralCirculo(){
        return xcCirc;
    }
    float getYcentralCirculo(){
        return ycCirc;
    }
    float getXcentralTrapezio(){
        return ycTrap;
    }
    float getYcentralTrapezio(){
        return xcTrap;
    }
    float getLarguraMenorTrapezio(){
        return largMenor;
    }
    float getLarguraMaiorTrapezio(){
        return largMaior;
    }
    float getRaioCirculo(){
        return raioCirc;
    }

    //setters

    void setXcentralCirculo(float newX){
        xcCirc = newX;
    }
    void setYcentralCirculo(float newY){
        ycCirc = newY;
    }
    void setXcentralTrapezio(float newX){
        xcTrap = newX;
    }
    void setYcentralTrapezio(float newY){
        ycTrap = newY;
    }
    void setLarguraMenorTrapezio(float newLarg){
        largMenor = newLarg;
    }
    void setLarguraMaiorTrapezio(float newLarg){
        largMaior = newLarg;
    }
    void setRaioCirculo(float newRaio){
        raioCirc = newRaio;
    }



};

#endif





