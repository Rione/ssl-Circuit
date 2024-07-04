#include "mainMode.hpp"

void MainMode::displaySet(UiKit *_ui){

  if(_ui->changeFlag_overMode){

    mainUI(_ui);

    _ui->changeFlag_overMode = false;
  }

  if(isTouched_state && millis() - isTouchedTime > 1000){
    display.setParttImage(120, 60, main_chgWhiteImg);
    display.publish(30, 150);
    isTouched_state = false;
  }else if(isTouched_kick && millis() - isTouchedTime > 1000){
    display.setParttImage(120, 60, main_kickWhiteImg);
    display.publish(170, 150);
    isTouched_kick = false;
  }

  // if(_ui->changeFlag_inMode && millis() - isTouchedTime > 1000){
  //   mainUI(_ui);
  //   _ui->changeFlag_inMode = false;
  // }

  // if(_ui->kickModeData.status.charge != _ui->kickModeData.status.chargePrev){
  //   if(_ui->kickModeData.status.charge == 0){
  //     display.setParttImage(260, 50, main_DischargeImg);
  //     display.publish(30, 70);
  //   }else if(_ui->kickModeData.status.charge == 1){
  //     display.setParttImage(260, 50, main_chargeImg);
  //     display.publish(30, 70);
  //   }

  //   _ui->kickModeData.status.chargePrev = _ui->kickModeData.status.charge;
  // }

  if(_ui->robotInfoData.status.charge != _ui->robotInfoData.chargePrev){
    if(_ui->robotInfoData.status.charge == 0){
      display.setParttImage(260, 50, main_DischargeImg);
      display.publish(30, 70);
    }else if(_ui->robotInfoData.status.charge == 1){
      display.setParttImage(260, 50, main_chargeImg);
      display.publish(30, 70);
    }
  }


  
  
}

void MainMode::determine(UiKit *_ui){
  //タッチ
  if(!touch.isTouched && touch.isTouchedPrev && !isTouched_state && !isTouched_kick){ 
    if(touch.point.x > 30 && touch.point.x < 150 && touch.point.y > 150 && touch.point.y < 210){

      //画面赤く
      display.setParttImage(120, 60, main_chgRedImg);
      display.publish(30, 150);
      //タッチした時間を記録
      isTouched_state = true;
      isTouchedTime = millis();

      //送信用にデータを変更
      _ui->modeData.status.charge_state = 1;
      _ui->stmSendSerial(&_ui->modeData); //送信
      _ui->modeData.status.charge_state = 0;
    
    }else if(touch.point.x > 170 && touch.point.x < 290 && touch.point.y > 150 && touch.point.y < 210){

      display.setParttImage(120, 60, main_kickRedImg);
      display.publish(170, 150);

      isTouched_kick = true;    
      isTouchedTime = millis();

      //送信用にデータを変更
      _ui->modeData.status.kick = 1;
      _ui->stmSendSerial(&_ui->modeData); //送信
      _ui->modeData.status.kick = 0;



    }
    

  }
}

void MainMode::mainUI(UiKit *_ui){
  display.createSprite();
  display.setBackgroundImage(main_kickImg);
  display.publish();
  
}
