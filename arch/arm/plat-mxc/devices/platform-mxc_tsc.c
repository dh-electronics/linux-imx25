
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_mxc_tsc_data_entry_single(soc, _id, _hwid, _size)		\
	{								\
		.id = _id,						\
		.iobase = soc ## _TSC ## _hwid ## _BASE_ADDR,		\
		.iosize = _size,					\
		.irq = soc ## _INT_TSC ## _hwid,			\
	}
#define imx_mxc_tsc_data_entry(soc, _id, _hwid, _size)			\
	[_id] = imx_mxc_tsc_data_entry_single(soc, _id, _hwid, _size)

#ifdef CONFIG_SOC_IMX21
const struct imx_mxc_tsc_data imx21_mxc_tsc_data __initconst =
	imx_mxc_tsc_data_entry_single(MX21, 0, , SZ_16K);
#endif /* ifdef CONFIG_SOC_IMX21 */

#ifdef CONFIG_SOC_IMX25
const struct imx_mxc_tsc_data imx25_mxc_tsc_data __initconst =
	imx_mxc_tsc_data_entry_single(MX25, 0, , SZ_16K);
#endif /* ifdef CONFIG_SOC_IMX25 */

#ifdef CONFIG_SOC_IMX27
const struct imx_mxc_tsc_data imx27_mxc_tsc_data __initconst =
	imx_mxc_tsc_data_entry_single(MX27, 0, , SZ_16K);
#endif /* ifdef CONFIG_SOC_IMX27 */

#ifdef CONFIG_SOC_IMX51
const struct imx_mxc_tsc_data imx51_mxc_tsc_data[] __initconst =
	imx_mxc_tsc_data_entry_single(MX51, 0, , SZ_4K);
};
#endif /* ifdef CONFIG_SOC_IMX51 */

struct platform_device *__init imx_add_tsc_imx(
		const struct imx_mxc_tsc_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("imx_adc", data->id,
			res, ARRAY_SIZE(res), NULL, 0);
}
