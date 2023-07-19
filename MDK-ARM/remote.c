
uint16_t pre_value = 0;
uint16_t cur_value = 0;
//uint16_t change =0;

#define REMOTE_ENABLE 172
#define REMOTE_DISABLE 1811

uint16_t IsChanged (uint16_t value)
{
	cur_value=value;
	if((cur_value>>4)==(pre_value>>4))
		return 0;
	else
		return 1;
}

void SigHandle(uint16_t curValue)
{
	if(IsChanged(curValue))
	{
		if(curValue == REMOTE_ENABLE)
		{
			send
		}
		if (curValue == REMOTE_DISABLE)
		{
			sent
		}
	}
}

ch1_value
uart_interuptt
{
	ch1_valee = ???;
}


while(1)
{
	....
	SigHandle(uint16_t ch1_value)
	......
{



