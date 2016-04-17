/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Japanese (Kanji and Kana)
 *
 * LCD Menu Messages
 * See also https://github.com/MarlinFirmware/Marlin/wiki/LCD-Language
 *
 */

#ifndef LANGUAGE_KANJI_AND_KANA_H
#define LANGUAGE_KANJI_AND_KANA_H

#define DISPLAY_CHARSET_ISO10646_KANJI_AND_KANA

// 漢字仮名交じり表示定義
#define WELCOME_MSG                         MACHINE_NAME _UxGT("準備完了")                // " ready."
#define MSG_SD_INSERTED                     _UxGT("カードが挿入されました")                    // "Card inserted"
#define MSG_SD_REMOVED                      _UxGT("カードがありません")                        // "Card removed"
#define MSG_LCD_ENDSTOPS                    _UxGT("エンドストップ")                            // "Endstops" // Max length 8 characters
#define MSG_MAIN                            _UxGT("メイン")                                 // "Main"
#define MSG_AUTOSTART                       _UxGT("自動開始")                              // "Autostart"
#define MSG_DISABLE_STEPPERS                _UxGT("モーター電源オフ")                          // "Disable steppers"
#define MSG_DEBUG_MENU                      _UxGT("デバッグメニュー")                           // "Debug Menu"
#define MSG_PROGRESS_BAR_TEST               _UxGT("プログレスバー 動作試験")                   // "Progress Bar Test"
#define MSG_AUTO_HOME                       _UxGT("原点復帰")                              // "Auto home"
#define MSG_AUTO_HOME_X                     _UxGT("X軸 原点復帰")                          // "Home X"
#define MSG_AUTO_HOME_Y                     _UxGT("Y軸 原点復帰")                          // "Home Y"
#define MSG_AUTO_HOME_Z                     _UxGT("Z軸 原点復帰")                          // "Home Z"
#define MSG_LEVEL_BED_HOMING                _UxGT("原点復帰中")                            // "Homing XYZ"
#define MSG_LEVEL_BED_WAITING               _UxGT("レベリング開始")                           //"Click to Begin"
#define MSG_LEVEL_BED_NEXT_POINT            _UxGT("次の測定点へ")                           // "Next Point"
#define MSG_LEVEL_BED_DONE                  _UxGT("レベリング完了")                           //"Leveling Done!"
#define MSG_LEVEL_BED_CANCEL                _UxGT("取止")                                  //"Cancel"
#define MSG_SET_HOME_OFFSETS                _UxGT("基準オフセット設定")                        // "Set home offsets"
#define MSG_HOME_OFFSETS_APPLIED            _UxGT("オフセットが適用されました")                   //"Offsets applied"
#define MSG_SET_ORIGIN                      _UxGT("基準セット")                              // "Set origin"
#define MSG_PREHEAT_1                       _UxGT("PLA 予熱")                              // "Preheat PLA"
#define MSG_PREHEAT_1_N                     MSG_PREHEAT_1 _UxGT(" ")
#define MSG_PREHEAT_1_ALL                   _UxGT("PLA 全て予熱")                           // " All"
#define MSG_PREHEAT_1_BEDONLY               _UxGT("PLA ベッド予熱")                          // "Bed"
#define MSG_PREHEAT_1_SETTINGS              MSG_PREHEAT_1 _UxGT("設定")                    // "conf"
#define MSG_PREHEAT_2                       _UxGT("ABS 予熱")                              // "Preheat ABS"
#define MSG_PREHEAT_2_N                     MSG_PREHEAT_2 _UxGT(" ")
#define MSG_PREHEAT_2_ALL                   _UxGT("ABS 全て予熱")                           // " All"
#define MSG_PREHEAT_2_BEDONLY               _UxGT("ABS ベッド予熱")                          // "Bed"
#define MSG_PREHEAT_2_SETTINGS              MSG_PREHEAT_2 _UxGT("設定")                    // "conf"
#define MSG_COOLDOWN                        _UxGT("加熱停止")                              // "Cooldown"
#define MSG_SWITCH_PS_ON                    _UxGT("電源オン")                               // "Switch power on"
#define MSG_SWITCH_PS_OFF                   _UxGT("電源オフ")                               // "Switch power off"
#define MSG_EXTRUDE                         _UxGT("押出")                                  // "Extrude"
#define MSG_RETRACT                         _UxGT("引込設定")                              // "Retract"
#define MSG_MOVE_AXIS                       _UxGT("軸移動")                                // "Move axis"
#define MSG_LEVEL_BED                       _UxGT("ベッドレベリング")                           // "Level bed"
#define MSG_MOVING                          _UxGT("移動中")                                // "Moving..."
#define MSG_FREE_XY                         _UxGT("XY軸 解放")                             // "Free XY"
#define MSG_MOVE_X                          _UxGT("X軸移動")                               // "Move X"
#define MSG_MOVE_Y                          _UxGT("Y軸移動")                               // "Move Y"
#define MSG_MOVE_Z                          _UxGT("Z軸移動")                               // "Move Z"
#define MSG_MOVE_E                          _UxGT("エクストルーダー")                           // "Extruder"
#define MSG_MOVE_01MM                       _UxGT("0.1mm移動")                              // "Move 0.1mm"
#define MSG_MOVE_1MM                        _UxGT("  1mm移動")                              // "Move 1mm"
#define MSG_MOVE_10MM                       _UxGT(" 10mm移動")                              // "Move 10mm"
#define MSG_SPEED                           _UxGT("スピード")                                 // "Speed"
#define MSG_BED_Z                           _UxGT("Zオフセット")                               // "Bed Z"
#define MSG_NOZZLE                          _UxGT("ノズル")                                   // "Nozzle"
#define MSG_BED                             _UxGT("ベッド")                                   // "Bed"
#define MSG_FAN_SPEED                       _UxGT("ファン速度")                               // "Fan speed"
#define MSG_FLOW                            _UxGT("吐出量")                                 // "Flow"
#define MSG_CONTROL                         _UxGT("制御")                                   // "Control"
#define MSG_MIN                             _UxGT(" ") LCD_STR_THERMOMETER _UxGT(" 最低")   // " Min"
#define MSG_MAX                             _UxGT(" ") LCD_STR_THERMOMETER _UxGT(" 最高")   // " Max"
#define MSG_FACTOR                          _UxGT(" ") LCD_STR_THERMOMETER _UxGT(" ファクター") // " Fact"
#define MSG_AUTOTEMP                        _UxGT("自動温度制御")                            // "Autotemp"
#define MSG_ON                              _UxGT("オン ")                                   // "On "
#define MSG_OFF                             _UxGT("オフ ")                                   // "Off"
#define MSG_PID_P                           _UxGT("PID-P")
#define MSG_PID_I                           _UxGT("PID-I")
#define MSG_PID_D                           _UxGT("PID-D")
#define MSG_PID_C                           _UxGT("PID-C")
#define MSG_SELECT                          _UxGT("選択")                                   // "Select"
#define MSG_ACC                             _UxGT("加速度 mm/s^2")                          // "Accel"
#define MSG_VX_JERK                         _UxGT("X軸躍度 mm/s")                           // "Vx-jerk"
#define MSG_VY_JERK                         _UxGT("Y軸躍度 mm/s")                           // "Vy-jerk"
#define MSG_VZ_JERK                         _UxGT("Z軸躍度 mm/s")                           // "Vz-jerk"
#define MSG_VE_JERK                         _UxGT("エクストルーダー躍度")                        // "Ve-jerk"
#define MSG_VMAX                            _UxGT("最大送り速度 ")                           // "Vmax "
#define MSG_VMIN                            _UxGT("最小送り速度")                            // "Vmin"
#define MSG_VTRAV_MIN                       _UxGT("最小移動速度")                           // "VTrav min"
#define MSG_AMAX                            _UxGT("最大加速度 ")                            // "Amax "
#define MSG_A_RETRACT                       _UxGT("引込加速度")                             // "A-retract"
#define MSG_A_TRAVEL                        _UxGT("移動加速度")                             // "A-travel"
#define MSG_XSTEPS                          _UxGT("Xsteps/mm")
#define MSG_YSTEPS                          _UxGT("Ysteps/mm")
#define MSG_ZSTEPS                          _UxGT("Zsteps/mm")
#define MSG_ESTEPS                          _UxGT("Esteps/mm")
#define MSG_E1STEPS                         _UxGT("E1steps/mm")
#define MSG_E2STEPS                         _UxGT("E2steps/mm")
#define MSG_E3STEPS                         _UxGT("E3steps/mm")
#define MSG_E4STEPS                         _UxGT("E4steps/mm")
#define MSG_TEMPERATURE                     _UxGT("温度")                                   // "Temperature"
#define MSG_MOTION                          _UxGT("動き設定")                                // "Motion"
#define MSG_VOLUMETRIC                      _UxGT("フィラメント")                               // "Filament"
#define MSG_VOLUMETRIC_ENABLED              _UxGT("E in mm3")
#define MSG_FILAMENT_DIAM                   _UxGT("フィラメント直径")                            // "Fil. Dia."
#define MSG_CONTRAST                        _UxGT("LCDコントラスト")                            // "LCD contrast"
#define MSG_STORE_EPROM                     _UxGT("メモリへ格納")                              // "Store memory"
#define MSG_LOAD_EPROM                      _UxGT("メモリから読み込み")                          // "Load memory"
#define MSG_RESTORE_FAILSAFE                _UxGT("設定リセット")                               // "Restore failsafe"
#define MSG_REFRESH                         _UxGT("リフレッシュ")                                // "Refresh"
#define MSG_WATCH                           _UxGT("情報画面")                                // "Info screen"
#define MSG_PREPARE                         _UxGT("準備設定")                                // "Prepare"
#define MSG_TUNE                            _UxGT("調整")                                    // "Tune"
#define MSG_PAUSE_PRINT                     _UxGT("一時停止")                                // "Pause print"
#define MSG_RESUME_PRINT                    _UxGT("プリント再開")                               // "Resume print"
#define MSG_STOP_PRINT                      _UxGT("プリント停止")                               // "Stop print"
#define MSG_CARD_MENU                       _UxGT("SDカードからプリント")                          // "Print from SD"
#define MSG_NO_CARD                         _UxGT("SDカードがありません")                          // "No SD card"
#define MSG_DWELL                           _UxGT("休止")                                     // "Sleep..."
#define MSG_USERWAIT                        _UxGT("暫くお待ち下さい")                             // "Wait for user..."
#define MSG_RESUMING                        _UxGT("プリント再開")                                // "Resuming print"
#define MSG_PRINT_ABORTED                   _UxGT("プリントが中止されました")                        // "Print aborted"
#define MSG_NO_MOVE                         _UxGT("動きません")                                  // "No move."
#define MSG_KILLED                          _UxGT("非常停止")                                  // "KILLED. "
#define MSG_STOPPED                         _UxGT("停止しました")                                // "STOPPED. "
#define MSG_CONTROL_RETRACT                 _UxGT("引込量 mm")                                 // "Retract mm"
#define MSG_CONTROL_RETRACT_SWAP            _UxGT("交換時引込量 mm")                           // "Swap Re.mm"
#define MSG_CONTROL_RETRACTF                _UxGT("引込速度 mm/s")                             // "Retract  V"
#define MSG_CONTROL_RETRACT_ZLIFT           _UxGT("ノズル退避量 mm")                             // "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         _UxGT("補償量 mm")                                 // "UnRet mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    _UxGT("交換時補償量 mm")                            // "S UnRet mm"
#define MSG_CONTROL_RETRACT_RECOVERF        _UxGT("補償速度 mm/s")                              // "UnRet  V"
#define MSG_AUTORETRACT                     _UxGT("自動引込")                                   // "AutoRetr."
#define MSG_FILAMENTCHANGE                  _UxGT("フィラメント交換")                               // "Change filament"
#define MSG_INIT_SDCARD                     _UxGT("SDカード再読込")                              // "Init. SD card"
#define MSG_CNG_SDCARD                      _UxGT("SDカード交換")                                // "Change SD card"
#define MSG_ZPROBE_OUT                      _UxGT("Zプローブ ベッド外")                             // "Z probe out. bed"
#define MSG_BLTOUCH_SELFTEST                _UxGT("BLTouch 自己診断")                           // "BLTouch Self-Test"
#define MSG_BLTOUCH_RESET                   _UxGT("BLTouch リセット")                             // "Reset BLTouch"
#define MSG_HOME                            _UxGT("先に")                                       // "Home" // Used as MSG_HOME " " MSG_X MSG_Y MSG_Z " " MSG_FIRST
#define MSG_FIRST                           _UxGT("を復帰させて下さい")                            // "first"
#define MSG_ZPROBE_ZOFFSET                  _UxGT("Zオフセット")                                   // "Z Offset"
#define MSG_BABYSTEP_X                      _UxGT("X軸微動")                                    // "Babystep X"
#define MSG_BABYSTEP_Y                      _UxGT("Y軸微動")                                    // "Babystep Y"
#define MSG_BABYSTEP_Z                      _UxGT("Z軸微動")                                    // "Babystep Z"
#define MSG_ENDSTOP_ABORT                   _UxGT("移動限界検出機能")                            // "Endstop abort"
#define MSG_HEATING_FAILED_LCD              _UxGT("加熱失敗")                                    // "Heating failed"
#define MSG_ERR_REDUNDANT_TEMP              _UxGT("エラー:冗長サーミスター機能")                       // "Err: REDUNDANT TEMP"
#define MSG_THERMAL_RUNAWAY                 _UxGT("熱暴走")                                      // "THERMAL RUNAWAY"
#define MSG_ERR_MAXTEMP                     _UxGT("エラー:設定最高温度超過")                        // "Err: MAXTEMP"
#define MSG_ERR_MINTEMP                     _UxGT("エラー:設定最低温度未満")                        // "Err: MINTEMP"
#define MSG_ERR_MAXTEMP_BED                 _UxGT("エラー:ベッド設定最高温度超過")                    // "Err: MAXTEMP BED"
#define MSG_ERR_MINTEMP_BED                 _UxGT("エラー:ベッド設定最低温度未満")                    // "Err: MINTEMP BED"
#define MSG_ERR_Z_HOMING                    MSG_HOME _UxGT(" ") MSG_X MSG_Y _UxGT(" ") MSG_FIRST // "G28 Z Forbidden"
#define MSG_HALTED                          _UxGT("プリンターは停止しました")                          // "PRINTER HALTED"
#define MSG_PLEASE_RESET                    _UxGT("リセットして下さい")                               // "Please reset"
#define MSG_SHORT_DAY                       _UxGT("d")                                           // One character only
#define MSG_SHORT_HOUR                      _UxGT("h")                                           // One character only
#define MSG_SHORT_MINUTE                    _UxGT("m")                                           // One character only
#define MSG_HEATING                         _UxGT("加熱中")                                      // "Heating..."
#define MSG_HEATING_COMPLETE                _UxGT("加熱完了")                                    // "Heating done."
#define MSG_BED_HEATING                     _UxGT("ベッド加熱中")                                  // "Bed Heating."
#define MSG_BED_DONE                        _UxGT("ベッド加熱完了")                                // "Bed done."
#define MSG_DELTA_CALIBRATE                 _UxGT("デルタ較正")                                    // "Delta Calibration"
#define MSG_DELTA_CALIBRATE_X               _UxGT("X軸較正")                                     // "Calibrate X"
#define MSG_DELTA_CALIBRATE_Y               _UxGT("Y軸較正")                                     // "Calibrate Y"
#define MSG_DELTA_CALIBRATE_Z               _UxGT("Z軸較正")                                     // "Calibrate Z"
#define MSG_DELTA_CALIBRATE_CENTER          _UxGT("中心較正")                                    // "Calibrate Center"
#define MSG_INFO_MENU                       _UxGT("このプリンターについて")                            // "About Printer"
#define MSG_INFO_PRINTER_MENU               _UxGT("プリンター情報")                                 // "Printer Info"
#define MSG_INFO_STATS_MENU                 _UxGT("プリント状況")                                  // "Printer Stats"
#define MSG_INFO_BOARD_MENU                 _UxGT("制御系情報")                                  // "Board Info"
#define MSG_INFO_THERMISTOR_MENU            _UxGT("サーミスター")                                   // "Thermistors"
#define MSG_INFO_EXTRUDERS                  _UxGT("エクストルーダー数")                              // "Extruders"
#define MSG_INFO_BAUDRATE                   _UxGT("ボーレート")                                    // "Baud"
#define MSG_INFO_PROTOCOL                   _UxGT("プロトコル")                                    // "Protocol"
#define MSG_LIGHTS_ON                       _UxGT("筐体内照明 点灯")                             //"Case light on"
#define MSG_LIGHTS_OFF                      _UxGT("筐体内照明 消灯")                             //"Case light off"
#define MSG_INFO_PRINT_COUNT                _UxGT("プリント数")                                    // "Print Count "
#define MSG_INFO_COMPLETED_PRINTS           _UxGT("完了数  ")                                    // "Finished    "
#define MSG_INFO_PRINT_TIME                 _UxGT("合計時間 ")                                   // "Total Time  "
#define MSG_INFO_PRINT_LONGEST              _UxGT("最長プリント時間")                               // "Longest job time"
#define MSG_INFO_PRINT_FILAMENT             _UxGT("フィラメント使用量累計")                           // "Extruded total"
#define MSG_INFO_MIN_TEMP                   _UxGT("設定最低温度")                                 // "Min Temp"
#define MSG_INFO_MAX_TEMP                   _UxGT("設定最高温度")                                 // "Max Temp"
#define MSG_INFO_PSU                        _UxGT("電源種別")                                     // "Power Supply"
#define MSG_DRIVE_STRENGTH                  _UxGT("モーター駆動力")                                 // "Drive Strength"
#define MSG_DAC_PERCENT                     _UxGT("DAC出力 %")                                    // "Driver %"
#define MSG_DAC_EEPROM_WRITE                MSG_STORE_EPROM                                       // "DAC EEPROM Write"
#define MSG_FILAMENT_CHANGE_HEADER          _UxGT("フィラメント交換")                                  // "CHANGE FILAMENT"
#define MSG_FILAMENT_CHANGE_OPTION_HEADER   _UxGT("動作を選択して下さい")                             // "CHANGE OPTIONS:"
#define MSG_FILAMENT_CHANGE_OPTION_EXTRUDE  _UxGT("更に押出す")                                     // "Extrude more"
#define MSG_FILAMENT_CHANGE_OPTION_RESUME   _UxGT("プリント再開")                                     // "Resume print"
#define MSG_FILAMENT_CHANGE_INIT_1          _UxGT("交換を開始します")                                 // "Wait for start"
#define MSG_FILAMENT_CHANGE_INIT_2          _UxGT("暫くお待ち下さい")                                  // "of the filament"
#define MSG_FILAMENT_CHANGE_UNLOAD_1        _UxGT("フィラメント抜出中")                                 // "Wait for"
#define MSG_FILAMENT_CHANGE_UNLOAD_2        _UxGT("暫くお待ち下さい")                                  // "filament unload"
#define MSG_FILAMENT_CHANGE_INSERT_1        _UxGT("フィラメントを挿入し、")                               // "Insert filament"
#define MSG_FILAMENT_CHANGE_INSERT_2        _UxGT("クリックすると続行します")                             // "and press button"
#define MSG_FILAMENT_CHANGE_LOAD_1          _UxGT("フィラメント装填中")                                 // "Wait for"
#define MSG_FILAMENT_CHANGE_LOAD_2          _UxGT("暫くお待ち下さい")                                  // "filament load"
#define MSG_FILAMENT_CHANGE_EXTRUDE_1       _UxGT("フィラメント押出中")                                 // "Wait for"
#define MSG_FILAMENT_CHANGE_EXTRUDE_2       _UxGT("暫くお待ち下さい")                                  // "filament extrude"
#define MSG_FILAMENT_CHANGE_RESUME_1        _UxGT("プリントを再開します")                                // "Wait for print"
#define MSG_FILAMENT_CHANGE_RESUME_2        _UxGT("暫くお待ち下さい")                                  // "to resume"

#endif // LANGUAGE_KANJI_AND_KANA_H
