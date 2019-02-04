double readAdcData(int channelNo){
    QFile adcCh("/sys/adcdriver/adcChannel");
    QString st =QString::number(channelNo);
    QByteArray ba = st.toLocal8Bit();
    const char * cstr;
    cstr = ba.data();
    adcCh.open(QIODevice::ReadWrite);
    if(adcCh.error() != QFile::NoError){
        qDebug() << "Could not open adcChannel driver";
        adcCh.close();
        return -10.0;
    }
    else{
        //            qDebug() << "Opened device opto driver successfully";
    }
    qint64 linelength = adcCh.write(cstr,st.size());
    if (linelength == -1){
        qDebug() << "Could not write to adcChannel";
        adcCh.close();
        return -10.0;
    }
    adcCh.close();


    QFile adcVolt("/sys/adcdriver/adcVoltage");

    adcVolt.open(QIODevice::ReadOnly);
    if(adcVolt.error() != QFile::NoError){
        qDebug() << "Could not open adcVoltage driver";
        adcVolt.close();
        return -10.0;
    }
    else{
        //            qDebug() << "Opened device adcVoltage driver successfully";
    }
    QTextStream in(&adcVolt);
    QString adcCount;
    adcCount = in.readLine();
    adcVolt.close();

    double adcVoltage=0;
    bool ok;
    int iadcCount = adcCount.toInt(&ok,10);
    if (iadcCount > 32767){
        iadcCount = ~iadcCount + 1;   // Take 2's compliment if number is negative
    }
    else{
    }
    adcVoltage = (5.0/32767.0) * iadcCount;
    // Division only applicable to E1 to E5 channels
    if (channelNo>=0 && channelNo < 5){
        adcVoltage = adcVoltage / 5.0;  //Divide by 5 since PGA gain of Instrumentation amplifier is 5
    }
    qDebug() << "adcChannel:" << st << "adcCount:" <<  adcCount << "adcVoltage:" <<  adcVoltage << "ok:" << ok;// << "iadcCount:" << iadcCount << ~iadcCount+1;// << cstr << ba.data();
    return adcVoltage;
}

void main(void){
	int i=0;
	while (1){
		readAdcData(i);
		i++;
		if (i>7){
			i=0;
		}
		mdelay(500); //sleep for 500mS
	}
}

