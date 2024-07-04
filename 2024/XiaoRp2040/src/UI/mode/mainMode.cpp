#include "mainMode.hpp"

void MainMode::displaySet(UiKit *_ui){

  if(_ui->changeFlag_overMode){

    mainUI(_ui);

    _ui->changeFlag_overMode = false;
  }

  // if(isTouched_state && millis() - isTouchedTime > 1000){
  //   display.setParttImage(120, 60, main_chgWhiteImg);
  //   display.publish(30, 150);
  //   isTouched_state = false;
  // }else if(isTouched_kick && millis() - isTouchedTime > 1000){
  //   display.setParttImage(120, 60, main_kickWhiteImg);
  //   display.publish(170, 150);
  //   isTouched_kick = false;
  // }

  if(_ui->changeFlag_inMode && millis() - isTouchedTime > 1000){
    mainUI(_ui);
    _ui->changeFlag_inMode = false;
  }

  
  
}

void MainMode::determine(UiKit *_ui){
  //タッチ
  if(!touch.isTouched && touch.isTouchedPrev && !_ui->changeFlag_inMode){ 
    if(touch.point.x > 30 && touch.point.x < 150 && touch.point.y > 150 && touch.point.y < 210){

      display.setParttImage(120, 60, main_chgRedImg);
      display.publish(30, 150);

      isTouched_state = true;
      _ui->changeFlag_inMode = true;
      isTouchedTime = millis();

      _ui->kickModeData.status.charge = 1;
    
    }else if(touch.point.x > 170 && touch.point.x < 290 && touch.point.y > 150 && touch.point.y < 210){

      display.setParttImage(120, 60, main_kickRedImg);
      display.publish(170, 150);

      isTouched_kick = true;
      _ui->changeFlag_inMode = true;      
      isTouchedTime = millis();

    }
    

  }
}

void MainMode::mainUI(UiKit *_ui){
  display.createSprite();
  display.setBackgroundImage(main_kickImg);
  display.publish();

  if(_ui->kickModeData.status.charge == 1){
    display.setParttImage(260, 50, main_chargeImg);
    display.publish(30, 70);
  }
  
}
