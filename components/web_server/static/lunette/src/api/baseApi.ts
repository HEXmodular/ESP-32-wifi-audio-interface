type SuccessResponse<T> = { success: true; data: T; error: null };
type ErrorResponse = { success: false; error: Error; data: null };
type ApiResponse<T> = SuccessResponse<T> | ErrorResponse;

export class BaseApi {
    private static baseUrl = import.meta.env.DEV ? 'https://lunette.local/api' : '/api';

    private static async handleResponse<T>(response: Response): Promise<ApiResponse<T>> {
        if (!response.ok) {
            return { success: false, error: new Error(`HTTP error! status: ${response.status}`), data: null };
        }
        try {
            return { success: true, data: await response.json() as T, error: null };
        } catch (error) {
            return { success: false, error: new Error('Failed to parse JSON response'), data: null };
        }
    }

    private static handleFetchError(error: unknown): ErrorResponse {
        if (error instanceof Error) {
            if (error.message === 'Failed to fetch') {
                return { success: false, error: new Error('Network error: Unable to connect to the server. Please check your connection.'), data: null };
            }
            return { success: false, error: error, data: null };
        }
        return { success: false, error: new Error('An unexpected error occurred'), data: null };
    }

    static async post<T>(endpoint: string, data: any): Promise<ApiResponse<T>> {
        try {
            const response = await fetch(`${this.baseUrl}/${endpoint}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(data)
            });
            return await this.handleResponse<T>(response);
        } catch (error) {
            return this.handleFetchError(error);
        }
    }

    static async get<T>(endpoint: string): Promise<ApiResponse<T>> {
        try {
            const response = await fetch(`${this.baseUrl}/${endpoint}`);
            return await this.handleResponse<T>(response);
        } catch (error) {
            return this.handleFetchError(error);
        }
        // return await Promise.resolve([] as T);
    }
}