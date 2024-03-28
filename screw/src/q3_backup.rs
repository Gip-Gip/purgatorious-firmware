use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, PartialEq, PartialOrd)]
pub struct Param {
    pub id: &'static str,
    pub val: Option<i16>,
    pub desc: &'static str,
}

#[derive(Debug, Clone, Deserialize, PartialEq, PartialOrd)]
pub struct ParamD {
    pub id: String,
    pub val: Option<i16>,
    pub desc: String,
}

pub const PARAMS: [Param; 231] = [
    Param {
        id: "0104",
        val: None,
        desc: "Offset",
    },
    Param {
        id: "0105",
        val: None,
        desc: "Inch Reference",
    },
    Param {
        id: "0106",
        val: None,
        desc: "Maximum Reference Forward",
    },
    Param {
        id: "0107",
        val: None,
        desc: "Minimum Reference Forward",
    },
    Param {
        id: "0108",
        val: None,
        desc: "Minimum Reference Reverse",
    },
    Param {
        id: "0109",
        val: None,
        desc: "Maximum Reference Reverse",
    },
    Param {
        id: "0110",
        val: None,
        desc: "Bipolar Reference Selector",
    },
    Param {
        id: "0111",
        val: None,
        desc: "Reference 'ON'",
    },
    Param {
        id: "0112",
        val: None,
        desc: "Reverse Selector",
    },
    Param {
        id: "0113",
        val: None,
        desc: "Inch Selector",
    },
    Param {
        id: "0114",
        val: None,
        desc: "Reference Select 1",
    },
    Param {
        id: "0115",
        val: None,
        desc: "Reference Select 2",
    },
    Param {
        id: "0116",
        val: None,
        desc: "Zero Reference Interlock",
    },
    Param {
        id: "0117",
        val: None,
        desc: "Reference 1",
    },
    Param {
        id: "0118",
        val: None,
        desc: "Reference 2",
    },
    Param {
        id: "0119",
        val: None,
        desc: "Reference 3",
    },
    Param {
        id: "0120",
        val: None,
        desc: "Reference 4",
    },
    Param {
        id: "0202",
        val: None,
        desc: "Ramp Enable",
    },
    Param {
        id: "0203",
        val: None,
        desc: "Ramp Hold",
    },
    Param {
        id: "0204",
        val: None,
        desc: "Forward Acceleration 1",
    },
    Param {
        id: "0205",
        val: None,
        desc: "Forward Deceleration 1",
    },
    Param {
        id: "0206",
        val: None,
        desc: "Reverse Deceleration 1",
    },
    Param {
        id: "0207",
        val: None,
        desc: "Reverse Acceleration 1",
    },
    Param {
        id: "0208",
        val: None,
        desc: "Forward Acceleration 2",
    },
    Param {
        id: "0209",
        val: None,
        desc: "Forward Deceleration 2",
    },
    Param {
        id: "0210",
        val: None,
        desc: "Reverse Deceleration 2",
    },
    Param {
        id: "0211",
        val: None,
        desc: "Reverse Acceleration 2",
    },
    Param {
        id: "0212",
        val: None,
        desc: "Inch Ramp Rate",
    },
    Param {
        id: "0213",
        val: None,
        desc: "Enable Inch Ramp",
    },
    Param {
        id: "0214",
        val: None,
        desc: "Forward Acceleration Selector",
    },
    Param {
        id: "0215",
        val: None,
        desc: "Forward Deceleration Selector",
    },
    Param {
        id: "0216",
        val: None,
        desc: "Reverse Deceleration Selector",
    },
    Param {
        id: "0217",
        val: None,
        desc: "Reverse Acceleration Selector",
    },
    Param {
        id: "0218",
        val: None,
        desc: "Common Ramp Selector",
    },
    Param {
        id: "0219",
        val: None,
        desc: "Ramp Rate x10",
    },
    Param {
        id: "0309",
        val: None,
        desc: "Speed Loop Kp",
    },
    Param {
        id: "0310",
        val: None,
        desc: "Speed Loop Ki",
    },
    Param {
        id: "0311",
        val: None,
        desc: "Speed Loop Kd",
    },
    Param {
        id: "0312",
        val: None,
        desc: "Digital Feedback Selector",
    },
    Param {
        id: "0313",
        val: None,
        desc: "AV Analog Feedback Selector",
    },
    Param {
        id: "0314",
        val: None,
        desc: "Feedback Encoder Scaling",
    },
    Param {
        id: "0315",
        val: None,
        desc: "Maximum Armature Voltage",
    },
    Param {
        id: "0316",
        val: None,
        desc: "Speed Readout Scaler",
    },
    Param {
        id: "0317",
        val: None,
        desc: "IR Compensation",
    },
    Param {
        id: "0318",
        val: None,
        desc: "Hard Speed Reference",
    },
    Param {
        id: "0319",
        val: None,
        desc: "Hard Speed Reference Selector",
    },
    Param {
        id: "0320",
        val: None,
        desc: "IR Droop Selector",
    },
    Param {
        id: "0321",
        val: None,
        desc: "Ramp Output Selector",
    },
    Param {
        id: "0322",
        val: None,
        desc: "Speed Offset Fine",
    },
    Param {
        id: "0323",
        val: None,
        desc: "Zero Speed Threshold",
    },
    Param {
        id: "0324",
        val: None,
        desc: "D-Term Source",
    },
    Param {
        id: "0325",
        val: None,
        desc: "Speed Error Filter",
    },
    Param {
        id: "0328",
        val: None,
        desc: "Speed Loop Prop Gain Multiplier",
    },
    Param {
        id: "0329",
        val: None,
        desc: "Reduce PI Gaines By 8",
    },
    Param {
        id: "0404",
        val: None,
        desc: "I Limit 1/Taper Start",
    },
    Param {
        id: "0405",
        val: None,
        desc: "I Limit 1 Bridge 1",
    },
    Param {
        id: "0406",
        val: None,
        desc: "I Limit Bridge 2",
    },
    Param {
        id: "0407",
        val: None,
        desc: "I Limit 2",
    },
    Param {
        id: "0408",
        val: None,
        desc: "Torque Reference",
    },
    Param {
        id: "0409",
        val: None,
        desc: "Current Offset",
    },
    Param {
        id: "0410",
        val: None,
        desc: "I Limit 2 Selector",
    },
    Param {
        id: "0411",
        val: None,
        desc: "Current Offset Selector",
    },
    Param {
        id: "0412",
        val: None,
        desc: "Mode Bit 0",
    },
    Param {
        id: "0413",
        val: None,
        desc: "Mode Bit 1",
    },
    Param {
        id: "0414",
        val: None,
        desc: "Quadrant 1 Enable",
    },
    Param {
        id: "0415",
        val: None,
        desc: "Quadrant 2 Enable",
    },
    Param {
        id: "0416",
        val: None,
        desc: "Quadrant 3 Enable",
    },
    Param {
        id: "0417",
        val: None,
        desc: "Quadrant 4 Enable",
    },
    Param {
        id: "0418",
        val: None,
        desc: "Enable Auto-I-Limit-Change",
    },
    Param {
        id: "0419",
        val: None,
        desc: "Current Limit Timer",
    },
    Param {
        id: "0420",
        val: None,
        desc: "Current Taper 1 Threshold",
    },
    Param {
        id: "0421",
        val: None,
        desc: "Current Taper 2 Threshold",
    },
    Param {
        id: "0422",
        val: None,
        desc: "Current Taper 1 Slope",
    },
    Param {
        id: "0423",
        val: None,
        desc: "Current Taper 2 Slope",
    },
    Param {
        id: "0504",
        val: None,
        desc: "Slew Rate Limit",
    },
    Param {
        id: "0505",
        val: None,
        desc: "Current Readout Scaler",
    },
    Param {
        id: "0506",
        val: None,
        desc: "Overload Threshold",
    },
    Param {
        id: "0507",
        val: None,
        desc: "Overload Time Heating",
    },
    Param {
        id: "0508",
        val: None,
        desc: "Overload Time Cooling",
    },
    Param {
        id: "0509",
        val: None,
        desc: "Enable Start-Up Auto-Tune",
    },
    Param {
        id: "0510",
        val: None,
        desc: "Reduced Endstop",
    },
    Param {
        id: "0512",
        val: None,
        desc: "Discontinuous Ki",
    },
    Param {
        id: "0513",
        val: None,
        desc: "Continuous Kp",
    },
    Param {
        id: "0514",
        val: None,
        desc: "Continuous Ki",
    },
    Param {
        id: "0515",
        val: None,
        desc: "Motor Constant",
    },
    Param {
        id: "0517",
        val: None,
        desc: "Inhibit Firing",
    },
    Param {
        id: "0518",
        val: None,
        desc: "Standstill Enable",
    },
    Param {
        id: "0519",
        val: None,
        desc: "Standstill Mode",
    },
    Param {
        id: "0520",
        val: None,
        desc: "Direct Firing Angle Control",
    },
    Param {
        id: "0521",
        val: None,
        desc: "Bridge Lockout Enable 4q12p",
    },
    Param {
        id: "0522",
        val: None,
        desc: "Disable Adaptive Control",
    },
    Param {
        id: "0523",
        val: None,
        desc: "Enable 1q12p",
    },
    Param {
        id: "0524",
        val: None,
        desc: "Series 12P Operation",
    },
    Param {
        id: "0525",
        val: None,
        desc: "Parallel 12P Operation",
    },
    Param {
        id: "0526",
        val: None,
        desc: "Extra-Safe Bridge Lockout",
    },
    Param {
        id: "0527",
        val: None,
        desc: "Continuous Autotune",
    },
    Param {
        id: "0528",
        val: None,
        desc: "Reduce Hysteresis On Bridge Changeover",
    },
    Param {
        id: "0529",
        val: None,
        desc: "Burden Resistor Change Bit",
    },
    Param {
        id: "0606",
        val: None,
        desc: "IR Compensation 2",
    },
    Param {
        id: "0607",
        val: None,
        desc: "Back EMF Set Point",
    },
    Param {
        id: "0608",
        val: None,
        desc: "Maximum Field Current",
    },
    Param {
        id: "0609",
        val: None,
        desc: "Maximum Field Current 1 (Full Field)",
    },
    Param {
        id: "0610",
        val: None,
        desc: "Minimum Field Current",
    },
    Param {
        id: "0611",
        val: None,
        desc: "Field Feedback Scaling 1",
    },
    Param {
        id: "0612",
        val: None,
        desc: "Field Economy Time-Out",
    },
    Param {
        id: "0613",
        val: None,
        desc: "Enable Field Control",
    },
    Param {
        id: "0614",
        val: None,
        desc: "Maximum Field 2 Selector",
    },
    Param {
        id: "0615",
        val: None,
        desc: "Enable Field Economy Time-Out",
    },
    Param {
        id: "0616",
        val: None,
        desc: "Field Current Loop Gain Selector",
    },
    Param {
        id: "0617",
        val: None,
        desc: "Voltage Loop Integral Gain",
    },
    Param {
        id: "0618",
        val: None,
        desc: "Enable Speed Gain Adjustment",
    },
    Param {
        id: "0619",
        val: None,
        desc: "Direct Firing Angle Control",
    },
    Param {
        id: "0620",
        val: None,
        desc: "Select Alternative IR Comp. 1",
    },
    Param {
        id: "0621",
        val: None,
        desc: "Firing Angle Front Endstop",
    },
    Param {
        id: "0622",
        val: None,
        desc: "Full Or Half Control",
    },
    Param {
        id: "0623",
        val: None,
        desc: "Reduce Gain By 2",
    },
    Param {
        id: "0624",
        val: None,
        desc: "Reduce Gain By 4",
    },
    Param {
        id: "0708",
        val: None,
        desc: "DAC 1 Source",
    },
    Param {
        id: "0709",
        val: None,
        desc: "DAC 2 Source",
    },
    Param {
        id: "0710",
        val: None,
        desc: "DAC 3 Source",
    },
    Param {
        id: "0711",
        val: None,
        desc: "GP1 Destination",
    },
    Param {
        id: "0712",
        val: None,
        desc: "GP2 Destination",
    },
    Param {
        id: "0713",
        val: None,
        desc: "GP3 Destination",
    },
    Param {
        id: "0714",
        val: None,
        desc: "GP4 Destination",
    },
    Param {
        id: "0715",
        val: None,
        desc: "Speed Destination",
    },
    Param {
        id: "0716",
        val: None,
        desc: "GP1 Scaling",
    },
    Param {
        id: "0717",
        val: None,
        desc: "GP2 Scaling",
    },
    Param {
        id: "0718",
        val: None,
        desc: "GP3 Scaling",
    },
    Param {
        id: "0719",
        val: None,
        desc: "GP4 Scaling",
    },
    Param {
        id: "0720",
        val: None,
        desc: "Speed Reference Scaling",
    },
    Param {
        id: "0721",
        val: None,
        desc: "DAC1 Scaling",
    },
    Param {
        id: "0722",
        val: None,
        desc: "DAC2 Scaling",
    },
    Param {
        id: "0723",
        val: None,
        desc: "DAC3 Scaling",
    },
    Param {
        id: "0724",
        val: None,
        desc: "Reference Encoder Scaling",
    },
    Param {
        id: "0725",
        val: None,
        desc: "Encoder Reference Selector",
    },
    Param {
        id: "0726",
        val: None,
        desc: "Current Input Selector",
    },
    Param {
        id: "0727",
        val: None,
        desc: "Current Sense Inverter",
    },
    Param {
        id: "0728",
        val: None,
        desc: "4mA Offset Selector",
    },
    Param {
        id: "0729",
        val: None,
        desc: "Invert Sign GP3, GP4",
    },
    Param {
        id: "0812",
        val: None,
        desc: "F2 Destination",
    },
    Param {
        id: "0813",
        val: None,
        desc: "F3 Destination",
    },
    Param {
        id: "0814",
        val: None,
        desc: "F4 Destination",
    },
    Param {
        id: "0815",
        val: None,
        desc: "F5 Destination",
    },
    Param {
        id: "0816",
        val: None,
        desc: "F6 Destination",
    },
    Param {
        id: "0817",
        val: None,
        desc: "F7 Destination",
    },
    Param {
        id: "0818",
        val: None,
        desc: "F8 Destination",
    },
    Param {
        id: "0819",
        val: None,
        desc: "F9 Destination",
    },
    Param {
        id: "0820",
        val: None,
        desc: "F10 Destination",
    },
    Param {
        id: "0821",
        val: None,
        desc: "Disable Normal Logic Functions",
    },
    Param {
        id: "0822",
        val: None,
        desc: "Invert F2 Input",
    },
    Param {
        id: "0823",
        val: None,
        desc: "Invert F3 Input",
    },
    Param {
        id: "0824",
        val: None,
        desc: "Invert F4 Input",
    },
    Param {
        id: "0825",
        val: None,
        desc: "Invert F5 Input",
    },
    Param {
        id: "0826",
        val: None,
        desc: "Invert F6 Input",
    },
    Param {
        id: "0827",
        val: None,
        desc: "Invert F7 Input",
    },
    Param {
        id: "0828",
        val: None,
        desc: "Invert F8 Input",
    },
    Param {
        id: "0829",
        val: None,
        desc: "Invert F9 Input",
    },
    Param {
        id: "0830",
        val: None,
        desc: "Invert F10 Input",
    },
    Param {
        id: "0831",
        val: None,
        desc: "Enable Inch Reverse",
    },
    Param {
        id: "0832",
        val: None,
        desc: "Enable Inch Forward",
    },
    Param {
        id: "0833",
        val: None,
        desc: "Enable Run Reverse",
    },
    Param {
        id: "0834",
        val: None,
        desc: "Enable Run Forward",
    },
    Param {
        id: "0907",
        val: None,
        desc: "Status 1 Source 1",
    },
    Param {
        id: "0908",
        val: None,
        desc: "Invert Status 1 Source 1",
    },
    Param {
        id: "0909",
        val: None,
        desc: "Status 1 Source 2",
    },
    Param {
        id: "0910",
        val: None,
        desc: "Invert Status 1 Source 2",
    },
    Param {
        id: "0911",
        val: None,
        desc: "Invert Status 1 Output",
    },
    Param {
        id: "0912",
        val: None,
        desc: "Status 1 Delay",
    },
    Param {
        id: "0913",
        val: None,
        desc: "Status 2 Source 1",
    },
    Param {
        id: "0914",
        val: None,
        desc: "Invert Status 2 Source 1",
    },
    Param {
        id: "0915",
        val: None,
        desc: "Status 2 Source 2",
    },
    Param {
        id: "0916",
        val: None,
        desc: "Invert Status 2 Source 2",
    },
    Param {
        id: "0917",
        val: None,
        desc: "Invert Status 2 Output",
    },
    Param {
        id: "0918",
        val: None,
        desc: "Status 2 Delay",
    },
    Param {
        id: "0919",
        val: None,
        desc: "Status 3 Source",
    },
    Param {
        id: "0920",
        val: None,
        desc: "Invert Status 3 Output",
    },
    Param {
        id: "0921",
        val: None,
        desc: "Status 4 Source",
    },
    Param {
        id: "0922",
        val: None,
        desc: "Invert Status 4 Output",
    },
    Param {
        id: "0923",
        val: None,
        desc: "Status 5 Source",
    },
    Param {
        id: "0924",
        val: None,
        desc: "Invert Status 5 Output",
    },
    Param {
        id: "0925",
        val: None,
        desc: "Status 6 Source",
    },
    Param {
        id: "0926",
        val: None,
        desc: "Invert Status 6 Output",
    },
    Param {
        id: "1029",
        val: None,
        desc: "Disable Field Loss",
    },
    Param {
        id: "1030",
        val: None,
        desc: "Disable Feedback Loss",
    },
    Param {
        id: "1031",
        val: None,
        desc: "Disable Phase Loss",
    },
    Param {
        id: "1032",
        val: None,
        desc: "Disable Motor Overtemp Trip",
    },
    Param {
        id: "1033",
        val: None,
        desc: "Disable Heatsink Overtemp Trip",
    },
    Param {
        id: "1034",
        val: None,
        desc: "External Trip",
    },
    Param {
        id: "1035",
        val: None,
        desc: "Processor 2 Trip",
    },
    Param {
        id: "1036",
        val: None,
        desc: "Disable Current Loop Loss Trip",
    },
    Param {
        id: "1037",
        val: None,
        desc: "Disable Armature Open Circuit Trip",
    },
    Param {
        id: "1101",
        val: None,
        desc: "Parameter 00.01",
    },
    Param {
        id: "1102",
        val: None,
        desc: "Parameter 00.02",
    },
    Param {
        id: "1103",
        val: None,
        desc: "Parameter 00.03",
    },
    Param {
        id: "1104",
        val: None,
        desc: "Parameter 00.04",
    },
    Param {
        id: "1105",
        val: None,
        desc: "Parameter 00.05",
    },
    Param {
        id: "1106",
        val: None,
        desc: "Parameter 00.06",
    },
    Param {
        id: "1107",
        val: None,
        desc: "Parameter 00.07",
    },
    Param {
        id: "1108",
        val: None,
        desc: "Parameter 00.08",
    },
    Param {
        id: "1109",
        val: None,
        desc: "Parameter 00.09",
    },
    Param {
        id: "1110",
        val: None,
        desc: "Parameter 00.10",
    },
    Param {
        id: "1111",
        val: None,
        desc: "Serial Address",
    },
    Param {
        id: "1112",
        val: None,
        desc: "Baud Rate",
    },
    Param {
        id: "1113",
        val: None,
        desc: "Serial Mode",
    },
    Param {
        id: "1117",
        val: None,
        desc: "Security Code 3",
    },
    Param {
        id: "1118",
        val: None,
        desc: "Boot-Up Parameter",
    },
    Param {
        id: "1119",
        val: None,
        desc: "Serial Programmable Source",
    },
    Param {
        id: "1120",
        val: None,
        desc: "Serial Scaling",
    },
    Param {
        id: "1121",
        val: None,
        desc: "LEDs byte",
    },
    Param {
        id: "1122",
        val: None,
        desc: "Disable Normal LED Functions",
    },
    Param {
        id: "1123",
        val: None,
        desc: "Permissive for MDA6, Rev. 3",
    },
    Param {
        id: "1124",
        val: None,
        desc: "Enable AC Line Dip Ride Through",
    },
    Param {
        id: "1203",
        val: None,
        desc: "Threshold 1 Source",
    },
    Param {
        id: "1204",
        val: None,
        desc: "Threshold 1 Level",
    },
    Param {
        id: "1205",
        val: None,
        desc: "Threshold 1 Hysteresis",
    },
    Param {
        id: "1206",
        val: None,
        desc: "Invert Threshold 1 Output",
    },
    Param {
        id: "1207",
        val: None,
        desc: "Threshold 1 Destination",
    },
    Param {
        id: "1208",
        val: None,
        desc: "Threshold 2 Source",
    },
    Param {
        id: "1209",
        val: None,
        desc: "Threshold 2 Level",
    },
    Param {
        id: "1210",
        val: None,
        desc: "Threshold 2 Hysteresis",
    },
    Param {
        id: "1211",
        val: None,
        desc: "Invert Threshold 2 Output",
    },
    Param {
        id: "1212",
        val: None,
        desc: "Threshold 2 Destination",
    },
    Param {
        id: "1306",
        val: None,
        desc: "Precision Reference, LSB",
    },
    Param {
        id: "1307",
        val: None,
        desc: "Precision Reference, MSB",
    },
    Param {
        id: "1308",
        val: None,
        desc: "Position Loop Gain",
    },
    Param {
        id: "1309",
        val: None,
        desc: "Position Loop Correction Limit",
    },
    Param {
        id: "1310",
        val: None,
        desc: "Enable Digital Lock",
    },
    Param {
        id: "1311",
        val: None,
        desc: "Rigid Lock Selector",
    },
    Param {
        id: "1312",
        val: None,
        desc: "Precision Reference Selector",
    },
    Param {
        id: "1313",
        val: None,
        desc: "Precision Reference Latch",
    },
    Param {
        id: "1314",
        val: None,
        desc: "Precision Speed Reference",
    },
];

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn verify_unique_entries() {
        for (i, param) in PARAMS.iter().enumerate() {
            assert_eq!(i, PARAMS.iter().position(|x| *x.id == *param.id).unwrap());
        }
    }
}
