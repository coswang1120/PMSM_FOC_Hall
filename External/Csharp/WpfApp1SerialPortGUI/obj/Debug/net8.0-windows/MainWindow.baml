   M S B A M L   `   `   `  ����  WpfApp1SerialPortGUI%    WpfApp1SerialPortGUI.MainWindow   5   	   W SPresentationCore, Version=8.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=8.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35e aSystem.Windows.Controls.Ribbon, Version=8.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089R NWindowsBase, Version=8.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35F 9http://schemas.microsoft.com/winfx/2006/xaml/presentation     5   	   R NSystem.Xaml, Version=8.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	     � Title$  串口通訊程式��5   	   $	��600��6   $	��800��6%   .��5      � $��10q�6    � RowDefinitions 5   
   �� 5      $
��Auto �6   �� 5      $��* �6   �� 5	      $
��Auto �6   �� 5
      $
��Auto �6   .+�5   
   �� $��0��6    �� Orientation$ 
Horizontal=�6"   $��0,0,0,10q�6;   .+�5      �� $��	端口：��6   $��Center=�6"   �� 5      -   5         ��Name$ PortComboBox��5      $	��100��6-   $	��5,0q�69   �� 5      -   5         ��Name$ RefreshButton��5      $��刷新��6,   $��60��69   $	��5,0q�6D   6Q   �� 5      -   5        $ ConnectButton��5      $��連接��6,   $��60��69   $	��5,0q�6D   6Q   �� 5      -   5        $ DisconnectButton��5      $��關閉��6/   $��60��6<   $	��5,0q�6G   6T   �� 5      -   5        $ ClearReceiveButton��5      $��清除接收��61   $��80��6@   $	��5,0q�6K   6X   �� 5      -   5        $ ClearSendButton��5      $��清除發送��6.   $��80��6=   $	��5,0q�6H   6U   5      �� 5   
   -   5         ��Name$ ReceiveTextBox��5      $��1��6*    �� 
IsReadOnly$
 True��5       �� TextWrapping$
 Wrap=�5      " �� VerticalScrollBarVisibility$
 Auto=�5      $��0,0,0,10q�5      �� 5    
   $��2��6   $ 
Horizontal=�6"   $��0,0,0,10q�6;   .+�5!      �� -   5        $ Command1Button��5!      $��Motor set 0��6-   $	��100��6C   $	��5,0q�6O   6\   �� 5"      -	   5        $ Command2Button��5"      $��Motor set 1��6-   $	��100��6C   $	��5,0q�6O   6\   �� 5#      -
   5        $ Command3Button��5#      $��Motor set 2��6-   $	��100��6C   $	��5,0q�6O   6\   5$      � 5'   
   $��3��6   	 � ColumnDefinitions	 5(      �� 5)      $��* �6#   �� 5*      $
��Auto �6#   .+�5,      �� -   5        $ SendTextBox��5,      $��0��6+   $��0,0,10,0q�6;   �� 5-      -   5        $ 
SendButton��5-      $��1��6)   $��發送��69   $��60��6F   6Q   5.      5/      50      