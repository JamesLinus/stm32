
#define DRV_REGISTER(elem) void* drv_elem_##elem __attribute__((__section__(".drv"))) = (void*)&elem
struct DRV_COMMON{
	int i;
};
struct DRV_COMMON     g_drv_common = {
	.i = 1,
};
DRV_REGISTER(g_drv_common);

int test(){
	g_drv_common.i = 10;
	return g_drv_common.i*2;
}
// end of file
