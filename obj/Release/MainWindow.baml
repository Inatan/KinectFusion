   M S B A M L   `   `   `  ����   KinectFusionBasics-WPF<    6Microsoft.Samples.Kinect.KinectFusionBasics.MainWindow   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     -     � Title$  Kinect Fusion TFCG��$	��735��$
��1100�� � 	Resources    MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� 
TargetType  ResetButton �� Property �� Value �� 
TargetType �� Orientation ��Name	 �� TextAlignment
 ��Name �� Triggers P� Property P� Value �� 
TargetName  SquareCheckBox �Name��%�  &	        &	      &	 *     &	 ?     7  7  &	 8    7  7  �� $ 	#ff6e6e6e���� $ 	#ff52318f���� $ 	#ff00BCF2���� " ��.���  � �� �� " ��.R�� .+��� $ 
Horizontal=�
������ .+��� $ 	ResetText��#��z��$
	 Left=�$��Center=�8��  $��15(�� $��9,0,0,0q�.+��� $
 ResetNormal��$��Images\ResetNormal.png��$
��None=�$��Center=��� $
 
ResetHover��$��Images\ResetHover.png��$
��None=�$��Center=�$y�	Collapsed=� P�  �@J�IsMouseOver . . ��� $ ResetNormal�� � y�$ 	Collapsed=��� $ 
ResetHover�� � y�$ Visible=��� 8  $ 	ResetText�� � ���� " ��.���  � �� �� " ��.R�� .+��� $ 
Horizontal=�
������ .+�� $ SquareCheckBoxChecked��.+��� $
 CheckedNormal��$��Images\CheckedNormal.png��$
��None=�$��Center=��� $
 CheckedHover��$��Images\CheckedHover.png��$
��None=�$��Center=�$y�	Collapsed=�� $ SquareCheckBoxUnchecked��$y�	Collapsed=�.+��� $
 UncheckedNormal��$ ��Images\UncheckedNormal.png��$
��None=�$��Center=��� $
 UncheckedHover��$��Images\UncheckedHover.png��$
��None=�$��Center=�$y�	Collapsed=��� $ SquareCheckBoxText��#��z��$
	 Left=�$��Center=�8��  $��15(�$��9,0,0,0q� P�  �@d�	IsChecked$ falseY�. ��� $ SquareCheckBoxChecked�� � y�$ 	Collapsed=��� $ SquareCheckBoxUnchecked�� � y�$ Visible=�P�  �@J�IsMouseOver . . ��� $ CheckedNormal�� � y�$ 	Collapsed=��� $ UncheckedNormal�� � y�$ 	Collapsed=��� $ CheckedHover�� � y�$ Visible=��� $ UncheckedHover�� � y�$ Visible=��� 8  $ SquareCheckBoxText�� � ��.��� -   $��
layoutGrid��$��	10 0 10 0q� � ColumnDefinitions �� $	��13* ��� $	��26* ��� $
��229* � � RowDefinitions �� $
��Auto ��� $��* ��� $
��Auto ��� $
��Auto �.+�]� $��0��$��0 0 0 20q�$��3��.+��� $��Images\Status.png��$
��None=�$��Center=�$��70 0 0 5q�(� $��1�� (� Stretch$ Uniform=�$
��Left=�$��3��.���� -   $��Image��$	��640��$	��480���� -   $��buttonResetReconstruction��$��1��#��[ $��Reset Reconstruction��$��35��$��Right=�$��Center=�$��0,437,26,139q�$��2���� -   $��	statusBar��$��3��$��Stretch=�$��Bottom=�
�������#��[  $��3��.@��� $��0 0 0 10q�.���� -   $��statusBarText��XVClick ‘Near Mode’ to change sensor range, and ‘Reset Reconstruction’ to clear!�� -    ��Name$ checkBoxNearMode��$��1��#��[ $��	Near Mode��$��39��$
��Left=�$��Center=�$��687,10,0,562q�$	��158��$��2���� -   $ checkBoxCaptureColor��$��1��#��[ $��Capture Color��$��36��$
��Left=�$��Center=�$��687,54,0,521q�$	��158��$��2���� $ 
Horizontal=�$��813,96,-5,474q�$��1��$��2��.+��� -	    ��Name$ txtNum��$��5,5,0,5q�$��50��$��0��$��18(� �� TextAlignment$ Center=��� -
    ��Name$ cmdUp��$��5,5,0,5q�$��˄��$��20���� -   $ cmdDown��$��0,5,0,5q�$��˅��$��20���� $ 
Horizontal=�$��814,145,-5,425q�$��1��$��2��.+��� -   $ txtNum1��$��5,5,0,5q�$��50��$��0��$��18(�$ Center=��� -   $ cmdUp1��$��5,5,0,5q�$��˄��$��20���� -   $ cmdDown1��$��0,5,0,5q�$��˅��$��20���� $ 
Horizontal=�$��815,192,-5,378q�$��1��$��2��.+��� -   $ txtNum2��$��5,5,0,5q�$��50��$��0��$��18(�$ Center=��� -   $ cmdUp2��$��5,5,0,5q�$��˄��$��20���� -   $ cmdDown2��$��0,5,0,5q�$��˅��$��20���� -   $ 	CreateObj��$��Save as OBJ��$
��Left=�$��744,376,0,0q�$��1��$	��Top=�$	��135��$��44��$��2���� $ 
Horizontal=�$��816,238,-5,332q�$��1��$��2��.+��� -   $ txtNum3��$��5,5,0,5q�$��50��$��0��$��18(�$ Center=��� -   $ cmdUp3��$��5,5,0,5q�$��˄��$��20���� -   $ cmdDown3��$��0,5,0,5q�$��˅��$��20���� $ 
Horizontal=�$��816,284,-5,286q�$��1��$��2��.+��� -   $ txtNum4��$��5,5,0,5q�$��50��$��0��$��18(�$ Center=��� -   $ cmdUp4��$��5,5,0,5q�$��˄��$��20���� -   $ cmdDown4��$��0,5,0,5q�$��˅��$��20���� $ 
Horizontal=�$��816,330,-5,240q�$��1��$��2��.+��� -   $ txtNum5��$��5,5,0,5q�$��50��$��0��$��18(�$ Center=��� -   $ cmdUp5��$��5,5,0,5q�$��˄��$��20���� -   $ cmdDown5��$��0,5,0,5q�$��˅��$��20���� -    ��Name$ label��$��X��$
��Left=�$��775,103,0,0q�$��1��$	��Top=�$��2���� -   $ 
label_Copy��$��Y��$
��Left=�$��775,152,0,0q�$��1��$	��Top=�$��2���� -   $ label_Copy1��$��Z��$
��Left=�$��775,199,0,0q�$��1��$	��Top=�$��2���� -   $ label_Copy2��$��Volume per Meter��$
��Left=�$��692,245,0,0q�$��1��$	��Top=�$��2���� -    $ label_Copy3��$��Depth Threshold Min��$
��Left=�$��687,337,0,0q�$��1��$	��Top=�$��2���� -!   $ label_Copy4��$��Depth Threshold Max��$
��Left=�$��687,288,0,0q�$��1��$	��Top=�$��2��