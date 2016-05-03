void setBacklight(byte brightness)
{
  lcd.write(0x53);  // send the backlight command
  lcd.write(brightness);  // send the brightness value
  delay(1);
}

void clearDisplay()
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x51);  // send the clear screen command
  delay(2);
}

void setLCDCursor(byte cursor_position)
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x45);  // send the set cursor command
  lcd.write(cursor_position);  // send the cursor position
  delay(1);
}

void sleepdisplay()
{
    lcd.write(0xFE);
    lcd.write(0x53);  // send the backlight command
    lcd.write(1);  // send the brightness value
    delay(1);
    lcd.write(0xFE);  //send special command
    lcd.write(0x42);  // Turn off display
    delay(1);
}
  
void wakedisplay()
{
    lcd.write(0xFE);
    lcd.write(0x53);  // send the backlight command
    lcd.write(4);  // send the brightness value
    delay(1);
    lcd.write(0xFE);  //send special command
    lcd.write(0x41);  //Turn on display
    delay(1);
}
  
void cleartoprow()
{
    setLCDCursor(0x00);
    delay(2);
    lcd.print("                ");
}

void clearbottomrow()
{
  setLCDCursor(0x40);
  delay(1);
  lcd.print("                ");
  delay(2);
}
