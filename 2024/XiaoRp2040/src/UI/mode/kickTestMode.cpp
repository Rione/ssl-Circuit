#include "kickTestMode.hpp"

void KickTestMode::displaySet(UiKit *_ui){

  if(_ui->changeFlag){

    kickUI();


    _ui->changeFlag = false;
  }
  
}

void KickTestMode::determine(UiKit *_ui){
  //タッチ
  if(!touch.isTouched && touch.isTouchedPrev){
    if(touch.point.x > 30 && touch.point.x < 150 && touch.point.y > 50 && touch.point.y < 100){

      // _ui->modeData.status.mode = 0;
      // _ui->changeFlag = true;

      if(_ui->kickModeData.mode == 0){
        display.setParttImage(120, 50, straightRedImg);
        display.publish(30, 50);
        _ui->kickModeData.mode = 1;
      }else if(_ui->kickModeData.mode == 1){
        display.setParttImage(120, 50, straightWhiteImg);
        display.publish(30, 50);
        _ui->kickModeData.mode = 0;
      }
      
      

    }
    
    
  }
}

void KickTestMode::kickUI(){
  display.createSprite(320, 240);
  display.setBackgroundImage(kickImg);
  display.publish();
  
}