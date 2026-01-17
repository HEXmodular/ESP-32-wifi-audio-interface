export interface AudioParamItem {
    name: string;
    shortName: string;
    audioParam: AudioParam;
}

export interface AudioParamList {
    hostName: string;
    audioParamList: AudioParamItem[];
}