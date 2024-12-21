void lcdStatus(int col,int row, char *label, float val1){
 
  lcd.setCursor(0,row);
  lcd.print(label);
  lcd.setCursor(col,row);
  lcd.print("      ");
  lcd.setCursor(col,row);
  lcd.print(val1);
  return;
}
